use crate::robo::io::lerp_rgb;
use crate::robo::{CHUNK_L, CHUNK_SIZE, Cell, ChunkCache, ScanPoints, to_world_beam_major};
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
use std::usize;
use std::{
    collections::HashMap,
    sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering::Relaxed},
    },
    thread::{self, JoinHandle},
};

const CHUNKCACHE_SIZE: usize = 10;
const MAX_RANGE_GUESS: f32 = 12.0;

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
    pub occ_thresh: f32,
    pub inv_2sigsqr: f32,
    pub z_hit: f32,
    pub z_rand: f32,
    pub rand_term: f32,
    pub miss_lp: f32,
}
impl OGMappingCore {
    fn searchspace_in_chunk(&self, chunk_pos: (i32, i32)) -> bool {
        const L: i32 = CHUNK_L as i32;
        self.search_distance >= chunk_pos.0
            && L - self.search_distance < chunk_pos.0
            && self.search_distance >= chunk_pos.1
            && L - self.search_distance < chunk_pos.1
    }
    #[inline]
    fn fast_search(
        &self,
        chunk_cache: &mut ChunkCache<CHUNKCACHE_SIZE>,
        map: &mut Map,
        hit_cell: (i32, i32),
        i_pos: (i32, i32),
        b_pos: (f32, f32),
    ) -> f32 {
        const L: i32 = CHUNK_L as i32;
        let r = self.search_distance;

        let chunk_key = (hit_cell.0.div_euclid(L), hit_cell.1.div_euclid(L));
        let chunk_ptr = chunk_cache.get(map, chunk_key);
        let chunk_ref = unsafe { &*chunk_ptr };
        let mut best_d2 = f32::INFINITY;

        for i in i_pos.0 - r..=i_pos.0 + r {
            for j in i_pos.1 - r..=i_pos.1 + r {
                let (i_idx, j_idx) = (i as usize, j as usize);
                let cell = chunk_ref[i_idx + j_idx * CHUNK_L];
                if !cell.occupied(self.occ_thresh) {
                    continue;
                }
                let ex = b_pos.0 - ((hit_cell.0 + i - i_pos.0) as f32 + cell.mx) * self.dx;
                let ey = b_pos.1 - ((hit_cell.1 + j - i_pos.1) as f32 + cell.my) * self.dx;
                best_d2 = best_d2.min(ex.powi(2) + ey.powi(2));
            }
        }
        best_d2
    }
    #[inline]
    fn slow_search(
        &self,
        chunk_cache: &mut ChunkCache<CHUNKCACHE_SIZE>,
        map: &mut Map,
        hit_cell: (i32, i32),
        b_pos: (f32, f32),
    ) -> f32 {
        const L: i32 = CHUNK_L as i32;
        let r = self.search_distance;
        let mut best_d2 = f32::INFINITY;
        for i in hit_cell.0 - r..=hit_cell.0 + r {
            for j in hit_cell.1 - r..=hit_cell.1 + r {
                let chunk_key = (i.div_euclid(L), j.div_euclid(L));
                let (ix, iy) = (i.rem_euclid(L) as usize, j.rem_euclid(L) as usize);
                let chunk_ptr = chunk_cache.get(map, chunk_key);
                let chunk_ref = unsafe { &*chunk_ptr };

                let cell = chunk_ref[ix + iy * CHUNK_L];
                if !cell.occupied(self.occ_thresh) {
                    continue;
                }
                let ex = b_pos.0 - (i as f32 + cell.mx) * self.dx;
                let ey = b_pos.1 - (j as f32 + cell.my) * self.dx;
                best_d2 = best_d2.min(ex.powi(2) + ey.powi(2));
            }
        }
        best_d2
    }
    #[inline]
    fn beam_lp(&self, best_d2: f32) -> f32 {
        if best_d2.is_finite() {
            (self.z_hit * (-best_d2 * self.inv_2sigsqr).exp() + self.rand_term).ln()
        } else {
            self.miss_lp
        }
    }
    fn single_lp(
        &self,
        pose: RobotPose,
        map: &mut Map,
        chunk_cache: &mut ChunkCache<CHUNKCACHE_SIZE>,
        scan_pts: &ScanPoints,
    ) -> f32 {
        const L: i32 = CHUNK_L as i32;

        let mut cum_lp = 0.0f32;
        for b_pos in to_world(&scan_pts.hits, pose) {
            let hit_cell = world2cell(b_pos[0], b_pos[1], self.dx);
            let (ix, iy) = (hit_cell.0.rem_euclid(L), hit_cell.1.rem_euclid(L));
            let best_d2 = if self.searchspace_in_chunk((ix, iy)) {
                self.fast_search(chunk_cache, map, hit_cell, (ix, iy), (b_pos[0], b_pos[1]))
            } else {
                self.slow_search(chunk_cache, map, hit_cell, (b_pos[0], b_pos[1]))
            };
            cum_lp += self.beam_lp(best_d2);
        }
        cum_lp
    }

    fn scanmatch_hillclimb(
        &self,
        prior: RobotPose,
        map: &mut Map,
        scan_pts: &ScanPoints,
    ) -> RobotPose {
        const L: i32 = CHUNK_L as i32;
        let mut chunk_cache: ChunkCache<CHUNKCACHE_SIZE> = ChunkCache::new();

        let mut stepsize_xy = self.xy_step;
        let mut stepsize_theta = self.theta_step;
        let mut n_halvings = 0;
        let mut n_steps = 0;
        let mut current_pose = prior;
        let mut current_lp = self.single_lp(current_pose, map, &mut chunk_cache, scan_pts);

        while n_halvings < self.max_halvings && n_steps < self.max_steps {
            // Setup test poses
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
            // Compute prob
            for (i, b_pos) in to_world_beam_major(&scan_pts.hits, &test_poses) {
                let hit_cell = world2cell(b_pos[0], b_pos[1], self.dx);
                let (ix, iy) = (hit_cell.0.rem_euclid(L), hit_cell.1.rem_euclid(L));

                let best_d2 = if self.searchspace_in_chunk((ix, iy)) {
                    self.fast_search(
                        &mut chunk_cache,
                        map,
                        hit_cell,
                        (ix, iy),
                        (b_pos[0], b_pos[1]),
                    )
                } else {
                    self.slow_search(&mut chunk_cache, map, hit_cell, (b_pos[0], b_pos[1]))
                };
                cand_lp[i] += self.beam_lp(best_d2);
            }
            // Update current pose
            let (best_cand, best_lp) = cand_lp
                .into_iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.total_cmp(b))
                .unwrap();
            if best_lp > current_lp {
                current_pose = test_poses[best_cand];
                current_lp = best_lp;
                n_steps += 1;
            } else {
                stepsize_xy *= 0.5;
                stepsize_theta *= 0.5;
                n_halvings += 1;
            }
        }
        current_pose
    }

    fn update_map(&self, map: &mut Map, pose: RobotPose, beams: &ScanPoints) {
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
    pub screenbuffer: Vec<u32>,
}

impl OGMapping {
    pub fn new(n_part: usize, dx: f32) -> Self {
        let z_hit = 0.95f32;
        let z_rand = 1.0 - z_hit;
        let rand_term = z_rand / MAX_RANGE_GUESS;
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
            occ_thresh: 0.1,
            inv_2sigsqr: (0.05f32).powi(-2) / 2.0,
            z_hit: z_hit,
            z_rand: z_rand,
            rand_term: rand_term,
            miss_lp: rand_term.ln(),
        };
        Self {
            core: core,
            particles: vec![RobotPose::default(); n_part],
            particle_maps: vec![Map::default(); n_part],
            weights: vec![1.0; n_part],
            last_imu: RobotPose::default(),
            last_update: 0.0,
            screenbuffer: vec![],
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
    pub fn render(&mut self) ->(usize,usize){
        let keys = self.particle_maps.iter().flat_map(|map| map.keys());
        let mut min: (i32, i32) = (i32::MAX, i32::MAX);
        let mut max: (i32, i32) = (i32::MIN, i32::MIN);
        for key in keys {
            min.0 = min.0.min(key.0);
            min.1 = min.1.min(key.1);
            max.0 = max.0.max(key.0);
            max.1 = max.1.max(key.1);
        }
        // Add offset to move min to origin
        let offset = (-min.0, -min.1);
        let size = ((max.0 - min.0) as usize, (max.1 - min.1) as usize);
        // Grey infill
        self.screenbuffer.resize(size.0 * size.1, 0x00808080);
        self.screenbuffer.fill(0x00808080);

        // draw all the maps
        for map in self.particle_maps.iter() {
            for (key, chunk) in map.iter() {
                let corner = ((key.0 + offset.0) as usize, (key.1 + offset.1) as usize);
                for i in 0..CHUNK_L {
                    for j in 0..CHUNK_L {
                        let activation = chunk[i + j * CHUNK_L].occupancy().clamp(0.0, 1.0)
                            / self.core.n_part as f32;
                        let activation = (activation * 255.0) as u32;
                        let color = activation << 16 | activation << 8 | activation;

                        self.screenbuffer[i + corner.0 + (corner.1 + j) * size.0] = (color);
                    }
                }
            }
        }
		// Here come the robots
        for pose in self.particles.iter() {
            let (s, c) = pose.theta.sin_cos();
            let line: Vec<(i32, i32)> = Bresenham::new(
                (
                    (pose.x/self.core.dx - c * 1.5 ) as i32,
                    (pose.y/ self.core.dx - s * 1.5 ) as i32,
                ),
                (
                    (pose.x/ self.core.dx + c * 1.5 ) as i32,
                    (pose.y/ self.core.dx + s * 1.5 ) as i32,
                ),
            )
            .collect();
            let n = line.len();
            for (i, (x, y)) in line.into_iter().enumerate() {
                let t = i as f32 / (n - 1).max(1) as f32;

                // ass -> nose
                let color = lerp_rgb(0x000000FF, 0x00FF0000, t);
				let pos:(usize,usize)=(
					(x+offset.0) as usize,
					(y+offset.1) as usize,
				);
				self.screenbuffer[pos.0+pos.1*size.0]=color;
				
            }
        }
		size
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
            while !shutdown_flag.load(Relaxed) {
			}
        })
    }
}
