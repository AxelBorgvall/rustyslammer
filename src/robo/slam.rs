use crate::robo::io::lerp_rgb;
use crate::robo::{CHUNK_L, CHUNK_SIZE, ChunkCache, ScanPoints, to_world_beam_major};
use crate::robo::{
    LidarScan, Map, RobotPose, SlamImage, bresenham::Bresenham, to_world, world2cell,
};
use arc_swap::ArcSwap;
use rand::{Rng, RngExt};
use rand_distr::{Distribution, Normal};
use rayon::iter::{IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator};
use std::time::Duration;
use std::time::Instant;
use std::{
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

    // Scanmatch search params
    pub xy_step: f32,
    pub max_steps: i32,
    pub theta_step: f32,
    pub max_halvings: i32,
    pub occ_thresh: f32,
    pub inv_2sigsqr: f32,
    pub z_hit: f32,
    pub _z_rand: f32,
    pub rand_term: f32,
    pub miss_lp: f32,
}
impl OGMappingCore {
    fn searchspace_in_chunk(&self, chunk_pos: (i32, i32)) -> bool {
        const L: i32 = CHUNK_L as i32;
        chunk_pos.0 >= self.search_distance
            && chunk_pos.0 < L - self.search_distance
            && chunk_pos.1 >= self.search_distance
            && chunk_pos.1 < L - self.search_distance
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
    ) -> (RobotPose, f32) {
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
        (current_pose, current_lp)
    }

    fn update_map(&self, map: &mut Map, pose: RobotPose, beams: &ScanPoints) {
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
            xy_step: dx,
            max_steps: 60,
            theta_step: 0.05,
            max_halvings: 4,
            occ_thresh: 0.1,
            inv_2sigsqr: (0.05f32).powi(-2) / 2.0,
            z_hit: z_hit,
            _z_rand: z_rand,
            rand_term: rand_term,
            miss_lp: rand_term.ln(),
        };
        Self {
            core: core,
            particles: vec![RobotPose::default(); n_part],
            particle_maps: vec![Map::default(); n_part],
            weights: vec![1.0; n_part],
            last_imu: RobotPose::default(),
            screenbuffer: vec![],
        }
    }

    pub fn update_positions(&mut self, imu_data: RobotPose, lidar_data: &LidarScan, dt: f32) {
        // Compute priors
        let delta = imu_data - self.last_imu;
        self.last_imu = imu_data;

        let mut rng = rand::rng();
        let noise_dist_x =
            Normal::new(0.0, (delta.x / dt * self.core.vel_noise).abs() + 0.005).unwrap();
        let noise_dost_y =
            Normal::new(0.0, (delta.y / dt * self.core.vel_noise).abs() + 0.005).unwrap();
        let noise_dist_theta =
            Normal::new(0.0, (delta.theta / dt * self.core.ang_noise).abs() + 0.01).unwrap();

        let scan_pts = ScanPoints::from_scan(lidar_data, lidar_data.max_distance * 0.98);
        let core = &self.core;

        self.particles
            .par_iter_mut()
            .zip(self.particle_maps.par_iter_mut())
            .zip(self.weights.par_iter_mut())
            .for_each(|((particle, map), weight)| {
                let mut rng = rand::rng();                let prior = RobotPose {
                    x: particle.x + noise_dist_x.sample(&mut rng),
                    y: particle.y + noise_dist_x.sample(&mut rng),
                    theta: particle.theta + noise_dist_theta.sample(&mut rng),
                } + delta;
                (*particle, *weight) = core.scanmatch_hillclimb(prior, map, &scan_pts);
            });
        let max = self
            .weights
            .iter()
            .cloned()
            .fold(f32::NEG_INFINITY, f32::max);
        let mut sum = 0.0;
        for w in self.weights.iter_mut() {
            *w = ((*w - max) / self.core.resampling_temp).exp();
            sum += *w;
        }
        let mean = sum / self.core.n_part as f32;
        let mut var = 0.0f32;
        for w in self.weights.iter_mut() {
            var += (*w - mean).powi(2);
            *w /= sum;
        }
        var /= self.core.n_part as f32;
        self.core.resampling_temp = 0.8 * self.core.resampling_temp + 0.20 * var.sqrt().max(1.0);
    }
    pub fn update_maps(&mut self, lidar_scan: &LidarScan) {
        let scan_pts = ScanPoints::from_scan(&lidar_scan, lidar_scan.max_distance * 0.98);
        for (pose, map) in self.particles.iter().zip(self.particle_maps.iter_mut()) {
            self.core.update_map(map, *pose, &scan_pts);
        }
    }
    pub fn resample(&mut self) {
        let n = self.core.n_part;
        let n_eff = 1.0f32 / self.weights.iter().map(|w| w * w).sum::<f32>();
        if n_eff >= n as f32 / 2.0 {
            return;
        }

        let r: f32 = rand::rng().random_range(0.0..1.0 / n as f32);
        let mut counts = vec![0usize; n];
        let (mut i, mut cum_weight) = (0, self.weights[0]);
        for k in 0..n {
            let ptr = r + k as f32 / n as f32;
            while cum_weight < ptr && i < n - 1 {
                i += 1;
                cum_weight += self.weights[i];
            }
            counts[i] += 1;
        }

        let old_poses = self.particles.clone();
        let old_maps = self.particle_maps.clone();
        let mut poses = Vec::with_capacity(n);
        let mut maps = Vec::with_capacity(n);

        for (i, (map, pose)) in old_maps.into_iter().zip(old_poses.into_iter()).enumerate() {
            let c = counts[i];
            for _ in 1..c {
                maps.push(map.clone());
                poses.push(pose);
            }
            if c > 0 {
                maps.push(map);
                poses.push(pose);
            }
        }
        self.particles = poses;
        self.particle_maps = maps;
        self.weights.fill(1.0 / n as f32);
    }
    pub fn render(&mut self) -> (usize, usize) {
        const L: i32 = CHUNK_L as i32;
        let keys = self.particle_maps.iter().flat_map(|map| map.keys());
        let mut min: (i32, i32) = (i32::MAX, i32::MAX);
        let mut max: (i32, i32) = (i32::MIN, i32::MIN);
        for key in keys {
            min.0 = min.0.min(key.0);
            min.1 = min.1.min(key.1);
            max.0 = max.0.max(key.0);
            max.1 = max.1.max(key.1);
        }
        if min.0 == i32::MAX {
            self.screenbuffer = vec![0; 4];

            return (1, 1);
        }
        // Add offset to move min to origin
        let offset = (-min.0, -min.1);
        let size = ((max.0 - min.0 + 1) as usize, (max.1 - min.1 + 1) as usize);
        // Grey infill

        // draw all the maps
        let w = size.0 * CHUNK_L;
        let h = size.1 * CHUNK_L;
        let mut activation = vec![0.5f32; w * h];
        let inv_n = 1.0f32 / self.core.n_part as f32;
        for map in self.particle_maps.iter() {
            for (key, chunk) in map.iter() {
                let corner = (
                    ((key.0 + offset.0) * L) as usize,
                    ((key.1 + offset.1) * L) as usize,
                );

                for i in 0..CHUNK_L {
                    for j in 0..CHUNK_L {
                        let occ = chunk[i + j * CHUNK_L].occupancy();
                        activation[(corner.0 + i) + (corner.1 + j) * w] += (occ * 0.5) * inv_n;
                    }
                }
            }
        }
        self.screenbuffer.resize(w * h, 0);
        for (dst, a) in self.screenbuffer.iter_mut().zip(&activation) {
            let v = (a.clamp(0.0, 1.0) * 255.0) as u32;
            *dst = v << 16 | v << 8 | v;
        }
        // Here come the robots
        for pose in self.particles.iter() {
            let (s, c) = pose.theta.sin_cos();
            let line: Vec<(i32, i32)> = Bresenham::new(
                (
                    (pose.x / self.core.dx - c * 1.5).floor() as i32,
                    (pose.y / self.core.dx - s * 1.5).floor() as i32,
                ),
                (
                    (pose.x / self.core.dx + c * 1.5).floor() as i32,
                    (pose.y / self.core.dx + s * 1.5).floor() as i32,
                ),
            )
            .collect();
            let n = line.len();
            for (i, (x, y)) in line.into_iter().enumerate() {
                let t = i as f32 / (n - 1).max(1) as f32;

                // ass -> nose
                let color = lerp_rgb(0x000000FF, 0x00FF0000, t);
                let pos: (usize, usize) =
                    ((x + offset.0 * L) as usize, (y + offset.1 * L) as usize);
                if pos.0 as usize >= w || pos.1 as usize >= h {
                    continue;
                }
                self.screenbuffer[pos.0 + pos.1 * size.0 * CHUNK_L] = color;
            }
        }
        (w, h)
    }
}

impl Slam for OGMapping {
    fn spawn(
        mut self,
        shutdown_flag: Arc<AtomicBool>,
        imu_in: Arc<RwLock<RobotPose>>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_out: Arc<ArcSwap<Map>>,
        img_out: Option<Arc<ArcSwap<SlamImage>>>,
    ) -> JoinHandle<()> {
        thread::spawn(move || {
            // Basic Slam loop goes here
            println!("Slam is starting now!");
            let mut last_upos = Instant::now() - Duration::from_secs_f32(0.2);
            while !shutdown_flag.load(Relaxed) {
                // Get imu and lidar
                let imu_state = {
                    let guard = imu_in.read().unwrap();
                    *guard
                };
                let lidar_data = lidar_in.load_full();

                // Update state
                let now = Instant::now();
                let dt = now.duration_since(last_upos).as_secs_f32();
                last_upos = now;
                println!("dt slam={dt}");
                self.update_positions(imu_state, &lidar_data, dt);
                self.update_maps(&lidar_data);
                self.resample();

                // Send map
                let (best, _) = self
                    .weights
                    .iter()
                    .enumerate()
                    .max_by(|(_, a), (_, b)| a.total_cmp(b))
                    .unwrap_or((0, &0.0));
                map_out.store(Arc::new(self.particle_maps[best].clone()));

                // Send render
                if let Some(mailbox) = &img_out {
                    let (nw, nh) = self.render();
                    let current_buffer = std::mem::take(&mut self.screenbuffer);
                    let new_frame = Arc::new(SlamImage {
                        data: current_buffer,
                        width: nw,
                        height: nh,
                    });

                    let old_frame_arc = mailbox.swap(new_frame);
                    match Arc::try_unwrap(old_frame_arc) {
                        Ok(old_frame) => {
                            self.screenbuffer = old_frame.data;
                        }
                        Err(_) => {
                            self.screenbuffer = vec![0; nw * nh];
                        }
                    }
                }
                if dt < 0.005 {
                    println!("sleepytime");
                    thread::sleep(Duration::from_millis(100));
                }
            }
        })
    }
}
