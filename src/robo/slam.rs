use crate::robo::{CHUNK_L, CHUNK_SIZE, Cell, ChunkCache, ScanPoints};
use crate::robo::{
    Chunk, LidarScan, Map, MapQuery, RobotPose, SlamImage, bresenham::Bresenham, cell_mut,
    to_world, world2cell,
};
use arc_swap::ArcSwap;
use rand::thread_rng;
use rand_distr::{Distribution, Normal};
use std::iter::Scan;
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;
use std::{
    collections::HashMap,
    sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering::Relaxed},
    },
    thread::{self, JoinHandle},
};

const CHUNKCACHE_SIZE: usize = 10;

pub trait Slam: Send {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        imu_in: Arc<RwLock<RobotPose>>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_out: Arc<ArcSwap<Map>>,
        img_out: Option<Arc<ArcSwap<SlamImage>>>,
    ) -> JoinHandle<()>;
}

pub struct OGMappingCore {
    // Params
    pub dx: f32,
    pub resampling_temp: f32,
    pub n_part: usize,
    pub ang_noise: f32,
    pub vel_noise: f32,
    pub search_distance: i32, // Radius searched around a lidar hit for terrain
    pub l_free: f32,
    pub l_occ: f32,

    // Scanmatch search params
    pub xy_step: f32,
    pub max_steps: i32,
    pub theta_step: f32,
    pub max_halvings: i32,
}
impl OGMappingCore {
    fn searchspace_in_chunk(&self, chunk_pos: (i32, i32)) -> bool {
        const L: i32 = CHUNK_L as i32;
        self.search_distance >= chunk_pos.0
            && L - self.search_distance < chunk_pos.0
            && self.search_distance >= chunk_pos.1
            && L - self.search_distance < chunk_pos.1
    }
    fn scanmatch_hillclimb(&mut self, prior: RobotPose, map: &mut Map, scan_pts: &ScanPoints) {
        const L: i32 = CHUNK_L as i32;
        let mut chunk_cache: ChunkCache<CHUNKCACHE_SIZE> = ChunkCache::new();

        let mut stepsize_xy = self.xy_step;
        let mut stepsize_theta = self.theta_step;
        let mut n_halvings = 0;
        let mut n_steps = 0;
        let mut current_pose = prior;
        let mut current_lp = 0.0f32;
		
		let world_pts=
        // Compute the lp of the current pose
        for b_pos in to_world(&scan_pts.hits, prior) {
            let hit_cell = world2cell(b_pos[0], b_pos[1], self.dx);
            let (ix, iy) = (hit_cell.0.rem_euclid(L), hit_cell.1.rem_euclid(L));
            let chunk_key = (hit_cell.0.div_euclid(L), hit_cell.1.div_euclid(L));
            let r = self.search_distance;
            // Fast track
            if self.searchspace_in_chunk((ix,iy)) {
                let chunk = chunk_cache.get(map, chunk_key);
				let mut highest_lp=f32::NEG_INFINITY;

                for i in ix - r..=ix + r {
                    for j in iy - r..=iy + r {
						let (i,j)=(i as usize,j as usize);
						
					}
                }
            }
        }

        while n_halvings < self.max_halvings && n_steps < self.max_steps {
            let mut cand_lp = [0.0f32; 6];
            let test_poses = [
                RobotPose {
                    x: current_pose.x + stepsize_xy,
                    ..current_pose
                },
                RobotPose {
                    x: current_pose.x - stepsize_xy,
                    ..current_pose
                },
                RobotPose {
                    y: current_pose.y + stepsize_xy,
                    ..current_pose
                },
                RobotPose {
                    y: current_pose.y - stepsize_xy,
                    ..current_pose
                },
                RobotPose {
                    theta: current_pose.theta + stepsize_theta,
                    ..current_pose
                },
                RobotPose {
                    theta: current_pose.theta - stepsize_theta,
                    ..current_pose
                },
            ];

            for b_pos in scan_pts.hits.iter() {}
        }
    }
    pub fn update_map(&self, map: &mut Map, pose: RobotPose, beams: &ScanPoints) {
        const L: i32 = CHUNK_L as i32;
        let start = world2cell(pose.x, pose.y, self.dx);

        let mut chunkcache: ChunkCache<CHUNKCACHE_SIZE> = ChunkCache::new();

        for [bx, by] in to_world(&beams.hits, pose) {
            let hit = world2cell(bx, by, self.dx);
            for (cx, cy) in Bresenham::new(start, hit) {
                if (cx, cy) == hit {
                    break;
                }
                chunkcache.add_free(map, (cx, cy));
            }

            chunkcache.add_hit(map, hit, (bx, by), self.dx);
        }

        for [bx, by] in to_world(&beams.free_only, pose) {
            let end = world2cell(bx, by, self.dx);
            for (cx, cy) in Bresenham::new(start, end) {
                chunkcache.add_free(map, (cx, cy));
            }
        }
    }
}

pub struct OGMapping {
    pub core: OGMappingCore,
    // State
    pub weights: Vec<f32>,
    pub particles: Vec<RobotPose>,
    pub particle_maps: Vec<Map>,
    pub last_imu: RobotPose,
    pub last_update: f32,
}

impl OGMapping {
    pub fn new(n_part: usize, dx: f32) -> Self {
        let core = OGMappingCore {
            dx: dx,
            resampling_temp: 8.0,
            n_part: n_part,
            ang_noise: 0.05,
            vel_noise: 0.05,
            search_distance: 1,
            l_free: 0.4,
            l_occ: 0.9,
            xy_step: dx,
            max_steps: 60,
            theta_step: 0.05,
            max_halvings: 4,
        };
        Self {
            core: core,
            particles: vec![RobotPose::default(); n_part],
            particle_maps: vec![Map::default(); n_part],
            weights: vec![1.0; n_part],
            last_imu: RobotPose::default(),
            last_update: 0.0,
        }
    }

    pub fn update_positions(&mut self, imu_data: RobotPose, lidar_data: &LidarScan, dt: f32) {
        // Compute priors
        let delta = (imu_data - self.last_imu);
        self.last_imu = imu_data;

        let mut rng = thread_rng();
        let noise_dist_x =
            Normal::new(0.0, (delta.x / dt * self.core.vel_noise).abs() + 0.005).unwrap();
        let noise_dost_y =
            Normal::new(0.0, (delta.y / dt * self.core.vel_noise).abs() + 0.005).unwrap();
        let noise_dist_theta =
            Normal::new(0.0, (delta.theta / dt * self.core.ang_noise).abs() + 0.01).unwrap();

        let priors = self.particles.iter().map(|particle| {
            RobotPose {
                x: particle.x + noise_dist_x.sample(&mut rng),
                y: particle.y + noise_dost_y.sample(&mut rng),
                theta: particle.theta + noise_dist_theta.sample(&mut rng),
            } + delta
        });
        let sigsqr: f32 = (0.05f32).powi(2);
        let scan_pts = ScanPoints::from_scan(lidar_data, lidar_data.max_distance * 0.98);

        for (i, prior) in priors.enumerate() {
            self.core
                .scanmatch_hillclimb(prior, &mut self.particle_maps[i], &scan_pts);
        }
    }
    pub fn update_maps(&mut self, lidar_scan: LidarScan) {
        let scan_pts = ScanPoints::from_scan(&lidar_scan, lidar_scan.max_distance * 0.98);
        for (pose, map) in self.particles.iter().zip(self.particle_maps.iter_mut()) {
            self.core.update_map(map, *pose, &scan_pts);
        }
    }
}

impl Slam for OGMapping {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        imu_in: Arc<RwLock<RobotPose>>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_out: Arc<ArcSwap<Map>>,
        img_out: Option<Arc<ArcSwap<SlamImage>>>,
    ) -> JoinHandle<()> {
        thread::spawn(move || {
            // Basic Slam loop goes here
            while !shutdown_flag.load(Relaxed) {}
        })
    }
}
