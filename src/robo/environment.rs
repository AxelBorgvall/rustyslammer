use crate::robo::{EnvImage, ImuState, LidarScan, Map, TwoWheelControl, bresenham::Bresenham, io};
use arc_swap::ArcSwap;
use minifb::Key::Y;
use std::net::Shutdown;
use std::thread::JoinHandle;
use std::{
    collections::HashMap,
    f32::consts::PI,
    sync::{Arc, RwLock, atomic::AtomicBool},
    thread,
};

pub trait Environment: Send {
    fn spawn(
        &self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_out: Arc<ArcSwap<LidarScan>>,
        control_in: Arc<RwLock<TwoWheelControl>>,
        img_out: Option<Arc<ArcSwap<EnvImage>>>,
    ) -> JoinHandle<()>;
}

pub struct SimEnv {
    // Robot
    pub n_rays: i32,
    pub max_range: f32,
    pub spread: f32,
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
    pub H: f32,
    pub W: f32,
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
    if (x < rad || y < rad) {
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
        if (x < 0.0 || y < 0.0) {
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
            n_rays: nrays,
            max_range: 8.0,
            spread: spread,
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
            H: ((nh as f32) / dx),
            W: ((nw as f32) / dx),
            dx,
            grid,
            dt: 0.02,
            seconds_per_iter: 0.2,
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

        for (x, y) in Bresenham::new(start_x, start_y, end_x, end_y) {
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
        self.v += (2.0) * (input.v_r.min(self.speed) - self.v) * self.dt;
        self.om += (2.0) * (input.om_r.min(self.angvel) - self.om) * self.dt;

        let x_prime = self.x + self.v * self.theta.cos() * self.dt;
        let y_prime = self.y + self.v * self.theta.sin() * self.dt;

        let mut path = Bresenham::new(
            self.real2idx(self.x),
            self.real2idx(self.y),
            self.real2idx(x_prime),
            self.real2idx(y_prime),
        );

        let mut target = (self.real2idx(self.x), self.real2idx(self.y));
        for (x, y) in path {
            target = (x, y);
            if self.grid[(x + y * self.nh as i32) as usize] {
                break;
            }
        }
        self.x = self.idx2real(target.0);
        self.y = self.idx2real(target.1);
        self.theta += self.om * self.dt;
    }
    pub fn render(&mut self) {}
}

impl Environment for SimEnv {
    fn spawn(
        &self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_out: Arc<ArcSwap<LidarScan>>,
        control_in: Arc<RwLock<TwoWheelControl>>,
        img_out: Option<Arc<ArcSwap<EnvImage>>>,
    ) -> JoinHandle<()> {
        thread::spawn(move || {})
    }
}
