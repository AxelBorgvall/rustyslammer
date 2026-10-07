use crate::robo::{EnvImage, LidarScan, RobotPose, TwoWheelControl, bresenham::Bresenham, io};
use crate::robo::{ScanPoints, to_world, world2cell};
use arc_swap::ArcSwap;
// use minifb::Key::Y;
// use std::net::Shutdown;
use std::sync::atomic::Ordering::Relaxed;
use std::thread::JoinHandle;
use std::time::Duration;
use std::{
    f32::consts::PI,
    sync::{Arc, RwLock, atomic::AtomicBool},
    thread,
};

pub trait Environment: Send {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_out: Arc<ArcSwap<LidarScan>>,
        imu_out: Arc<RwLock<RobotPose>>,
        control_in: Arc<RwLock<TwoWheelControl>>,
        img_out: Option<Arc<ArcSwap<EnvImage>>>,
    ) -> JoinHandle<()>;
}

pub struct SimEnv {
    // Robot
    pub _n_rays: i32,
    pub max_range: f32,
    pub _spread: f32,
    pub speed: f32,
    pub angvel: f32,
    pub angles: Vec<f32>,

    // Pose
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub v: f32,
    pub om: f32,

    // Map
    pub nh: usize,
    pub nw: usize,
    pub _h: f32,
    pub _w: f32,
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

impl SimEnv {
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
            _n_rays: nrays,
            max_range: 12.0,
            _spread: spread,
            speed: 0.2,
            angvel: 0.2,
            angles: angles,
            x,
            y,
            theta: theta,
            om: 0.0,
            v: 0.0,
            nh,
            nw,
            _h: ((nh as f32) / dx),
            _w: ((nw as f32) / dx),
            dx,
            grid,
            dt: 0.02,
            seconds_per_iter: 0.02,
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
        let end_x = ((rx + self.max_range * ray_angle.cos()) / self.dx) as i32;
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
        let ranges: Vec<f32> = self
            .angles
            .iter()
            .map(|&angle| self.cast_ray(self.x, self.y, self.theta + angle))
            .collect();

        LidarScan {
            ranges,
            angles: self.angles.clone(),
            max_distance: self.max_range,
        }
    }

    // Kinematics
    pub fn step_fwd(&mut self, input: TwoWheelControl) {
        self.v += 2.0 * (input.v_r.clamp(-self.speed, self.speed) - self.v) * self.dt;
        self.om += 2.0 * (input.om_r.clamp(-self.angvel, self.angvel) - self.om) * self.dt;

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
            let x = x as usize;
            let y = y as usize;
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

impl Environment for SimEnv {
    fn spawn(
        mut self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_out: Arc<ArcSwap<LidarScan>>,
        imu_out: Arc<RwLock<RobotPose>>,
        control_in: Arc<RwLock<TwoWheelControl>>,
        img_out: Option<Arc<ArcSwap<EnvImage>>>,
    ) -> JoinHandle<()> {
        thread::spawn(move || {
            // Main simulation loop
            let mut count: u32 = 0;
            println!("Environment thread starting now");

            while !shutdown_flag.load(Relaxed) {
                let current_control = {
                    let guard = control_in.read().unwrap();
                    *guard
                };
                self.step_fwd(current_control);

                count += 1;
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

                // Publish environment render
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

                thread::sleep(Duration::from_secs_f32(self.seconds_per_iter));
            }
        })
    }
}
