use core::f32;
use std::{
    f32::consts::PI,
    sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering::Relaxed},
    },
    thread::{self, JoinHandle},
    time::Instant,
};

use arc_swap::ArcSwap;
use rand::thread_rng;
use rand_distr::{Distribution, Normal};

use crate::robo::{
    EnvImage, LidarScan, Pos, RobotPose, ScanPoints, TwoDOFControl, bresenham::Bresenham,
    environment::Environment, io, to_world, world2cell,
};

pub struct CarEnv {
    // Robot
    pub max_speed: f32,
    pub max_whang: f32,
    pub robo_w: f32,
    pub robo_l: f32,
    pub wh_w: f32,
    pub wh_l: f32,

    // Pose
    pub pose: RobotPose, // Located between rear wheels
    pub v: f32,
    pub whang: f32,

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

    // Sim speed
    pub dt: f32,
    pub seconds_per_iter: f32,

    // buffers
    pub screenbuffer: Vec<u32>,
}

impl CarEnv {
    pub fn new(path: &str) -> Self {
        let (nh, nw, dx, grid) =
            io::load_data(path).expect("Failed to load the Map from the path specified.");
        let nh = nh as usize;
        let nw = nw as usize;
        let w = nw as f32 * dx;
        let h = nh as f32 * dx;

        let pose = RobotPose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };

        let nrays = 360;
        let spread = 2.0 * PI;
        let nrays_f = nrays as f32;
        let half_spread = spread / 2.0;
        let angles: Vec<f32> = (0..nrays)
            .map(|i| (i as f32) / (nrays_f - 1.0) * spread - half_spread)
            .collect();

        let mut this = Self {
            max_speed: 0.0,
            max_whang: 45.0 * (PI / 180.0),
            robo_w: 0.2,
            robo_l: 0.4,
            wh_w: 0.15,
            wh_l: 0.35,
            pose: RobotPose {
                x: w * 0.5,
                y: h * 0.5,
                theta: 0.0,
            },
            v: 0.0,
            whang: 0.0,
            angles,
            max_range: 12.0,
            n_rays: nrays,
            spread,
            const_nosie: 0.01,
            noise_factor: 0.005,
            ang_noise: 0.002,
            nh,
            nw,
            h,
            w,
            dx,
            grid,
            dt: 0.02,
            seconds_per_iter: 0.02,
            screenbuffer: vec![0; nh * nw],
        };

        // Find free position for robo
        this.pose = {
            let mut search_rad: f32 = 0.0;
            let mut search_ang: f32 = 0.0;
            let mut prop_pose = this.pose;
            let mut found = false;
            let dtheta = 0.02f32;
            while !this
                .corners(prop_pose)
                .iter()
                .all(|p| p.x >= 0.0 && p.x < this.w && p.y >= 0.0 && p.y < this.h)
            {
                if !this.collision(pose) {
                    found = true;
                    break;
                } else {
                    // after 2 pi we want L_car diff in rad to search nicely.
                    search_ang += dtheta;
                    search_rad += (this.robo_l / dtheta) / (2.0 * PI);
                    let (s, c) = search_ang.sin_cos();
                    prop_pose = RobotPose {
                        x: this.pose.x + c * search_rad,
                        y: this.pose.y + s * search_ang,
                        theta: this.pose.theta,
                    }
                }
            }
            if found {
                prop_pose
            } else {
                panic!("Could not find a place for the robo. Sorry.")
            }
        };

        this
    }

    #[inline]
    fn corners(&self, pose: RobotPose) -> [Pos; 4] {
        let (s, c) = pose.theta.sin_cos();
        let l_offset = self.robo_l * 0.5 - self.wh_l * 0.5;
        let corner = Pos {
            x: pose.x - l_offset * c - self.robo_w * 0.5 * s,
            y: pose.y - l_offset * s + self.robo_w * 0.5 * c,
        };
        let delta_1 = Pos { x: s, y: -c } * self.robo_w;
        let delta_2 = Pos { x: c, y: s } * self.robo_l;

        [
            corner,
            corner + delta_1,
            corner + delta_1 + delta_2,
            corner + delta_2,
        ]
    }
    fn collision(&self, pose: RobotPose) -> bool {
        // Bresenham along all 4 sides of the cars outer body
        let corners = self.corners(self.pose);

        let mut edges = [
            Bresenham::new(
                self.reals2idx(corners[0].astup()),
                self.reals2idx(corners[1].astup()),
            ),
            Bresenham::new(
                self.reals2idx(corners[1].astup()),
                self.reals2idx(corners[2].astup()),
            ),
            Bresenham::new(
                self.reals2idx(corners[2].astup()),
                self.reals2idx(corners[3].astup()),
            ),
            Bresenham::new(
                self.reals2idx(corners[3].astup()),
                self.reals2idx(corners[0].astup()),
            ),
        ];
        edges.into_iter().any(|mut e| {
            e.any(|(x, y)| {
                x >= 0
                    && y >= 0
                    && self
                        .grid
                        .get(x as usize + y as usize * self.nw)
                        .copied()
                        .unwrap_or(false)
            })
        })
    }
    fn real2idx(&self, x: f32) -> i32 {
        return (x / self.dx) as i32;
    }
    fn reals2idx(&self, x: (f32, f32)) -> (i32, i32) {
        ((x.0 / self.dx) as i32, (x.1 / self.dx) as i32)
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
        let mut rng = thread_rng();
        let ang_noise = Normal::new(0.0, self.ang_noise).unwrap();
        let norm = Normal::new(0.0, 1.0).unwrap();

        let ranges: Vec<f32> = self
            .angles
            .iter()
            .map(|angle| {
                let angle = angle + ang_noise.sample(&mut rng);
                let dist = self.cast_ray(self.pose.x, self.pose.y, self.pose.theta + angle);
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
    pub fn step_fwd(&mut self, input: TwoDOFControl) {}

    fn drawline(&mut self, line: Bresenham, color: u32) {
        for (x, y) in line {
            let x = (x as usize).clamp(0, self.nw - 1);
            let y = (y as usize).clamp(0, self.nh - 1);
            self.screenbuffer[x + y * self.nw] = color;
        }
    }
    fn draw_box(&mut self, outline: [Bresenham; 4], color: u32) {
        let mut outline: Vec<(i32, i32)> = outline.into_iter().flatten().collect();
        outline.sort_unstable_by_key(|p| (p.1, p.0));
        for row in outline.chunk_by(|a, b| a.1 == b.1) {
            let minx = row.first().unwrap().0 as usize;
            let maxx = row.last().unwrap().0 as usize;
            let y = row[0].1 as usize;
			for x in minx..maxx{
				self.screenbuffer[x+self.nw*y]=color;
			}
        }
    }

    pub fn render(&mut self, lidar_data: Option<&LidarScan>) {
        self.screenbuffer.resize(self.nw * self.nh, 0);
        for (pixel, &is_wall) in self.screenbuffer.iter_mut().zip(self.grid.iter()) {
            *pixel = if is_wall { 0x00000000 } else { 0xFFFFFFFF };
        }

        if let (Some(scan)) = lidar_data {
            let start = (self.real2idx(self.pose.x), self.real2idx(self.pose.y));
            let scan_pts = ScanPoints::from_scan(scan, scan.max_distance);
            for ray in to_world(
                &scan_pts.hits,
                RobotPose {
                    x: self.pose.x,
                    y: self.pose.y,
                    theta: self.pose.theta,
                },
            ) {
                let end = (self.real2idx(ray[0]), self.real2idx(ray[1]));
                let line = Bresenham::new(start, end);
                self.drawline(line, 0x00D0D0FF);
            }
            for ray in to_world(
                &scan_pts.free_only,
                RobotPose {
                    x: self.pose.x,
                    y: self.pose.y,
                    theta: self.pose.theta,
                },
            ) {
                let end = (self.real2idx(ray[0]), self.real2idx(ray[1]));
                let line = Bresenham::new(start, end);
                self.drawline(line, 0x00D0D0FF);
            }
        }

		let corns:[(i32,i32);4]=self.corners(self.pose).map(|pos| self.reals2idx((pos.x,pos.y)));
		let edges=[
			Bresenham::new(corns[0],corns[1]),
			Bresenham::new(corns[1],corns[2]),
			Bresenham::new(corns[2],corns[3]),
			Bresenham::new(corns[3],corns[0]),
		];
		self.draw_box(edges, 0x00FF8000);

        let x_offset = self.real2idx(self.pose.x + self.pose.theta.cos() * self.robo_l*1.2 * self.dx);
        let y_offset = self.real2idx(self.pose.y + self.pose.theta.sin() * self.robo_l*1.2 * self.dx);
        let nose = Bresenham::new(
            (self.real2idx(self.pose.x), self.real2idx(self.pose.y)),
            (x_offset, y_offset),
        );

        self.drawline(nose, 0x00FF000000);
    }
}

impl Environment for CarEnv {
    fn spawn(
        mut self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_out: Arc<ArcSwap<LidarScan>>,
        imu_out: Arc<RwLock<RobotPose>>,
        control_in: Arc<RwLock<TwoDOFControl>>,
        img_out: Option<Arc<ArcSwap<EnvImage>>>,
    ) -> JoinHandle<()> {
        thread::spawn(move || {
            println!("Environment thread starting now");

            let mut lastcall = Instant::now();
            let mut lastdraw = Instant::now();

            while !shutdown_flag.load(Relaxed) {
                // Step env
                // publish lidar
                // publish imu
                // publish render
            }
        })
    }
}
