use crate::robo::{EnvImage, ImuState, LidarScan, Map, TwoWheelControl};
use arc_swap::ArcSwap;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::net::Shutdown;
use std::thread::JoinHandle;
use std::{
    collections::HashMap,
    f32::consts::PI,
    sync::{Arc, RwLock, atomic::AtomicBool},
    thread,
};

pub fn save_grid(path: &str, height: u32, width: u32, dx: f32, data: &[bool]) -> io::Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    writer.write_all(&height.to_le_bytes())?;
    writer.write_all(&width.to_le_bytes())?;
    writer.write_all(&dx.to_le_bytes())?;
    let len = data.len() as u64;
    writer.write_all(&len.to_le_bytes())?;

    for &val in data {
        writer.write_all(&[val as u8])?;
    }
    Ok(())
}

pub fn load_data(path: &str) -> io::Result<(u32, u32, f32, Vec<bool>)> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    let mut buf_4 = [0u8; 4];
    let mut buf_8 = [0u8; 8];

    reader.read_exact(&mut buf_4)?;
    let height = u32::from_le_bytes(buf_4);

    reader.read_exact(&mut buf_4)?;
    let width = u32::from_le_bytes(buf_4);

    reader.read_exact(&mut buf_4)?;
    let dx = f32::from_le_bytes(buf_4);

    reader.read_exact(&mut buf_8)?;
    let len = u64::from_le_bytes(buf_8) as usize;

    let mut data = Vec::with_capacity(len);
    let mut buf_1 = [0u8; 1]; // Buffer for reading 1 byte at a time

    for _ in 0..len {
        reader.read_exact(&mut buf_1)?;
        data.push(buf_1[0] != 0);
    }

    Ok((height, width, dx, data))
}

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

    pub x: f32,
    pub y: f32,
    pub theta: f32,

    // Map
    pub nh: usize,
    pub nw: usize,
    pub H: f32,
    pub W: f32,
    pub dx: f32,
    pub grid: Vec<bool>,

    // Buffers
    distbuffer: Vec<f32>,
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
            load_data(path).expect("Failed to load the Map from the path specified.");
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
            nh,
            nw,
            H: ((nh as f32) / dx),
            W: ((nw as f32) / dx),
            dx,
            grid,
            distbuffer: vec![0.0; nrays as usize],
        }
    }

    pub fn lidarscan(&mut self) -> LidarScan {
        let rx = self.x;
        let ry = self.y;
        let rtheta = self.theta;

        let start_x = (rx / self.dx) as i32;
        let start_y = (ry / self.dx) as i32;

        let width = self.nw as i32;
        let height = self.nh as i32;
        // Run bresenham for every ray
        for i in 0..self.n_rays as usize {
            let mut x0 = start_x;
            let mut y0 = start_y;
            let ray_angle = rtheta + self.angles[i];
            let end_x_f = rx + self.max_range * ray_angle.cos();
            let end_y_f = ry + self.max_range * ray_angle.sin();

            let end_x = (end_x_f / self.dx) as i32;
            let end_y = (end_y_f / self.dx) as i32;

            let delta_x = (start_x - end_x).abs();
            let delta_y = -(start_y - end_y).abs();
            let sx = (start_x - end_x).signum();
            let sy = -(start_y - end_y).signum();
            let mut err = delta_x + delta_y;

            loop {
				// Register hits or OOB
                if x0 < 0 || x0 >= width || y0 < 0 || y0 >= height {
                    self.distbuffer[i] = self.max_range;
                    break;
                }
                if self.grid[(y0 * width + x0) as usize] {
                    let hit_x_f = (x0 as f32 * self.dx) + (self.dx / 2.0);
                    let hit_y_f = (x0 as f32 * self.dx) + (self.dx / 2.0);
                    let dist = ((hit_x_f - rx).powi(2) + (hit_x_f - rx).powi(2)).sqrt();
                    self.distbuffer[i] = dist.min(self.max_range);
                    break;
                }
				
				if x0==start_x && y0==start_y{
					self.distbuffer[i]= self.max_range;
				}
				
				// step forward
				let e2=2*err;
				if e2>delta_y{
					if x0==end_x{
						self.distbuffer[i]=self.max_range;
						break;
					}
					err+=delta_y;
					x0+=sx;
				}
				if e2>delta_x{
					if y0==end_y{
						self.distbuffer[i]=self.max_range;
						break;
					}
					err+=delta_x;
					y0+=sy;
				}

            }
        }

        LidarScan {
            ranges: vec![],
            angles: vec![],
            max_distance: self.max_range,
        }
    }

    pub fn step_fwd(&mut self, dt: f32) {}
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
