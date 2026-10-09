use crate::robo::environment::Environment;
use crate::robo::{EnvImage, LidarScan, RobotPose, TwoDOFControl, bresenham::Bresenham, io};
use crate::robo::{ScanPoints, to_world, world2cell};
use rand::Rng;
use rand::rngs::ThreadRng;
use rand_distr::{Distribution, Normal};

use arc_swap::ArcSwap;
// use minifb::Key::Y;
// use std::net::Shutdown;
use std::sync::atomic::Ordering::Relaxed;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use std::{
    f32::consts::PI,
    sync::{Arc, RwLock, atomic::AtomicBool},
    thread,
};

pub struct BasicSimEnv {
    // Robot
    pub speed: f32,
    pub angvel: f32,

    // Pose
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub v: f32,
    pub om: f32,

    // Lidar
    pub angles: Vec<f32>,
    pub max_range: f32,
    pub n_rays: i32,
    pub spread: f32,
    pub const_nosie: f32,
    pub noise_factor: f32,
    pub ang_noise: f32,

    // Map
    pub nh: usize,
    pub nw: usize,
    pub h: f32,
    pub w: f32,
    pub dx: f32,
    pub grid: Vec<bool>,

    // Speed
    pub dt: f32,
    pub seconds_per_iter: f32,

    // buffers
    pub screenbuffer: Vec<u32>,
}

/* ------------------------------ Helper funcs ------------------------------ */
fn checkaround(grid: &Vec<bool>, nh: usize, nw: usize, x: usize, y: usize, rad: usize) -> bool {
    if (x + rad + 1 > nw) || (y + rad + 1 > nh) {
        return true;
    }
    if x < rad || y < rad {
        return true;
    }
    for i in x - rad..x + rad {
        for j in y - rad..y + rad {
            if grid[i + j * nw] {
                return true;
            }
        }
    }
    false
}

impl BasicSimEnv {
    pub fn new(path: &str) -> Self {
        let (nh, nw, dx, grid) =
            io::load_data(path).expect("Failed to load the Map from the path specified.");
        let nh = nh as usize;
        let nw = nw as usize;

        // Place the robot
        let freespace = (0.5 / dx) as usize;
        let mut x: f32 = -1.0;
        let mut y: f32 = -1.0;
        let theta = 0.0;
        'outer: for i in (0..nw).step_by(5) {
            for j in (0..nh).step_by(5) {
                if !(checkaround(&grid, nh, nw, i, j, freespace)) {
                    x = i as f32 * dx;
                    y = j as f32 * dx;
                    break 'outer;
                };
            }
        }

        if x < 0.0 || y < 0.0 {
            panic!("We could not find a place for the robo. Sorry :(")
        };

        let nrays = 360;
        let spread = 2.0 * PI;
        let nrays_f = nrays as f32;
        let half_spread = spread / 2.0;
        let angles: Vec<f32> = (0..nrays)
            .map(|i| (i as f32) / (nrays_f - 1.0) * spread - half_spread)
            .collect();

        Self {
            // Robot
            speed: 1.0,
            angvel: 1.4,

            // Pose
            x: x,
            y: y,
            theta: theta,
            v: 0.0,
            om: 0.0,

            // Lidar
            angles: angles,
            max_range: 12.0,
            n_rays: nrays,
            spread: spread,
            const_nosie: 0.01,
            noise_factor: 0.005,
            ang_noise: 0.002,

            // Map
            nh: nh,
            nw: nw,
            h: ((nh as f32) / dx),
            w: ((nw as f32) / dx),
            dx: dx,
            grid: grid,

            // Speed
            dt: 0.02,
            seconds_per_iter: 0.02,

            // buffers
            screenbuffer: vec![0; nh * nw],
        }
    }
    fn real2idx(&self, x: f32) -> i32 {
        return (x / self.dx) as i32;
    }
    fn idx2real(&self, x: i32) -> f32 {
        return x as f32 * self.dx;
    }

    // Lidar simualtion
    fn cast_ray(&self, rx: f32, ry: f32, ray_angle: f32) -> f32 {
        let start_x = (rx / self.dx) as i32;
        let start_y = (ry / self.dx) as i32;
        let end_x = ((rx + self.max_range * (ray_angle).cos()) / self.dx) as i32;
        let end_y = ((ry + self.max_range * ray_angle.sin()) / self.dx) as i32;

        for (x, y) in Bresenham::new((start_x, start_y), (end_x, end_y)) {
            if x < 0 || x >= self.nw as i32 || y < 0 || y >= self.nh as i32 {
                return self.max_range;
            }

            if self.grid[(y * self.nw as i32 + x) as usize] {
                let hit_x = (x as f32 * self.dx) + (self.dx / 2.0);
                let hit_y = (y as f32 * self.dx) + (self.dx / 2.0);
                let dist = ((hit_x - rx).powi(2) + (hit_y - ry).powi(2)).sqrt();
                return dist.min(self.max_range);
            }
        }

        self.max_range
    }
    pub fn lidarscan(&self) -> LidarScan {
        let mut rng = rand::rng();
        let ang_noise = Normal::new(0.0, self.ang_noise).unwrap();
        let norm = Normal::new(0.0, 1.0).unwrap();

        let ranges: Vec<f32> = self
            .angles
            .iter()
            .map(|angle| {
                let angle = angle + ang_noise.sample(&mut rng);
                let dist = self.cast_ray(self.x, self.y, self.theta + angle);
                let noise_scale = (self.const_nosie + dist * self.noise_factor);
                dist + norm.sample(&mut rng) * noise_scale
            })
            .collect();

        LidarScan {
            ranges,
            angles: self.angles.clone(),
            max_distance: self.max_range,
        }
    }

    // Kinematics
    pub fn step_fwd(&mut self, input: TwoDOFControl) {
        self.v += 4.0 * (input.trans_r.clamp(-self.speed, self.speed) - self.v) * self.dt;
        self.om += 4.0 * (input.rot_r.clamp(-self.angvel, self.angvel) - self.om) * self.dt;

        let x_prime = self.x + self.v * self.theta.cos() * self.dt;
        let y_prime = self.y + self.v * self.theta.sin() * self.dt;

        let path = Bresenham::new(
            (self.real2idx(self.x), self.real2idx(self.y)),
            (self.real2idx(x_prime), self.real2idx(y_prime)),
        );

        let mut target = (self.real2idx(self.x), self.real2idx(self.y));
        let mut hit_wall = false;

        for (x, y) in path {
            if self.grid[(x + y * self.nw as i32) as usize] {
                hit_wall = true;
                break;
            }
            target = (x, y);
        }

        if hit_wall {
            self.x = self.idx2real(target.0);
            self.y = self.idx2real(target.1);
            self.v = 0.0;
        } else {
            self.x = x_prime;
            self.y = y_prime;
        }
        self.theta += self.om * self.dt;
    }
    fn drawline(&mut self, line: Bresenham, color: u32) {
        for (x, y) in line {
            let x = (x as usize).clamp(0, self.nw - 1);
            let y = (y as usize).clamp(0, self.nh - 1);
            self.screenbuffer[x + y * self.nw] = color;
        }
    }
    pub fn render(&mut self, lidar_data: Option<&LidarScan>) {
        self.screenbuffer.resize(self.nw * self.nh, 0);
        for (pixel, &is_wall) in self.screenbuffer.iter_mut().zip(self.grid.iter()) {
            *pixel = if is_wall { 0x00000000 } else { 0xFFFFFFFF };
        }

        if let (Some(scan)) = lidar_data {
            let start = (self.real2idx(self.x), self.real2idx(self.y));
            let scan_pts = ScanPoints::from_scan(scan, scan.max_distance);
            for ray in to_world(
                &scan_pts.hits,
                RobotPose {
                    x: self.x,
                    y: self.y,
                    theta: self.theta,
                },
            ) {
                let end = (self.real2idx(ray[0]), self.real2idx(ray[1]));
                let line = Bresenham::new(start, end);
                self.drawline(line, 0x00D0D0FF);
            }
            for ray in to_world(
                &scan_pts.free_only,
                RobotPose {
                    x: self.x,
                    y: self.y,
                    theta: self.theta,
                },
            ) {
                let end = (self.real2idx(ray[0]), self.real2idx(ray[1]));
                let line = Bresenham::new(start, end);
                self.drawline(line, 0x00D0D0FF);
            }
        }

        let cx = self.real2idx(self.x) as usize;
        let cy = self.real2idx(self.y) as usize;
        let xrange = cx.saturating_sub(2)..(cx + 2).min(self.nw);
        let yrange = cy.saturating_sub(2)..(cy + 2).min(self.nh);

        for y in yrange {
            for x in xrange.clone() {
                self.screenbuffer[x + y * self.nw] = 0x00FF0000;
            }
        }

        let x_offset = self.real2idx(self.x + self.theta.cos() * 6.0 * self.dx);
        let y_offset = self.real2idx(self.y + self.theta.sin() * 6.0 * self.dx);
        let nose = Bresenham::new(
            (self.real2idx(self.x), self.real2idx(self.y)),
            (x_offset, y_offset),
        );

        self.drawline(nose, 0x00FF000000);
    }
}

impl Environment for BasicSimEnv {
    fn spawn(
        mut self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_out: Arc<ArcSwap<LidarScan>>,
        imu_out: Arc<RwLock<RobotPose>>,
        control_in: Arc<RwLock<TwoDOFControl>>,
        img_out: Option<Arc<ArcSwap<EnvImage>>>,
    ) -> JoinHandle<()> {
        thread::spawn(move || {
            // Main simulation loop
            println!("Environment thread starting now");

            let mut lastcall = Instant::now();
            let mut lastdraw = Instant::now();

            while !shutdown_flag.load(Relaxed) {
                let current_control = {
                    let guard = control_in.read().unwrap();
                    *guard
                };
                self.step_fwd(current_control);

                // Publish lidardata
                let scan = self.lidarscan();
                lidar_out.store(Arc::new(scan.clone()));

                // Publish IMUData
                {
                    let mut mtx = imu_out.write().unwrap();
                    *mtx = RobotPose {
                        x: self.x,
                        y: self.y,
                        theta: self.theta,
                    };
                }

                let now = Instant::now();
                let dt = lastcall - now;
                lastcall = now;
                if dt.as_secs_f32() < self.seconds_per_iter {
                    thread::sleep(Duration::from_secs_f32(self.seconds_per_iter) - dt);
                }

                // Publish environment render
                let now = Instant::now();
                let dt_draw = now - lastdraw;
                if dt_draw > Duration::from_millis(80) {
                    lastdraw = now;
                    self.render(Some(&scan));
                    if let Some(mailbox) = &img_out {
                        let current_buffer = std::mem::take(&mut self.screenbuffer);
                        let new_frame = Arc::new(EnvImage {
                            data: current_buffer,
                            width: self.nw,
                            height: self.nh,
                        });

                        let old_frame_arc = mailbox.swap(new_frame);
                        match Arc::try_unwrap(old_frame_arc) {
                            Ok(old_frame) => {
                                self.screenbuffer = old_frame.data;
                            }
                            Err(_) => {
                                self.screenbuffer = vec![0; self.nw * self.nh];
                            }
                        }
                    }
                }
            }
        })
    }
}
