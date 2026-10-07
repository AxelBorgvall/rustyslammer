use crate::robo::{ LidarScan, Map, TwoWheelControl};
use arc_swap::ArcSwap;
use std::{ f32::consts::PI, sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering::Relaxed},
    }, thread::{self, JoinHandle}, time::Duration,
};

pub trait Controller: Send {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_in: Arc<ArcSwap<Map>>,
        ctrl_out: Arc<RwLock<TwoWheelControl>>,
    ) -> JoinHandle<()>;
}

pub struct BasicController {
    pub speed: f32,
    pub angvel: f32,
    pub reactrange: f32,
    pub k: f32,
}

impl Default for BasicController {
    fn default() -> Self {
        Self {
            speed: 1.0,
            angvel: 1.2,
            reactrange: 4.0,
            k: 1.0,
        }
    }
}

impl BasicController {
    fn control(&self, scan: &LidarScan) -> TwoWheelControl {
        let mut fx = 0.0f32;
        let mut fy = 0.0f32;

        let max_dist = (scan.max_distance - 0.1).min(self.reactrange);

		let mut nhits=0;
        for (&angle, &range) in scan.angles.iter().zip(scan.ranges.iter()) {
            if range > max_dist || angle.abs() > PI {
                continue;
            }

            let mag = self.k / (range.powi(2) + 0.01);
            fx -= mag * angle.cos();
            fy -= mag * angle.sin();
			nhits+=1;

        }
        if nhits > 0 {
			let nhits=nhits as f32;
            fx /= nhits;
            fy /= nhits;
        }
        fx += self.speed;

        let heading_r = fy.atan2(fx);
        let omega = heading_r.clamp(-self.angvel, self.angvel);
        let v = fx.clamp(-self.speed, self.speed);

        TwoWheelControl {
            v_r: v,
            om_r: omega,
        }
    }
}

impl Controller for BasicController {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_in: Arc<ArcSwap<Map>>,
        ctrl_out: Arc<RwLock<TwoWheelControl>>,
    ) -> JoinHandle<()> {
        thread::spawn(move || {
            while !shutdown_flag.load(Relaxed) {
                let lidardata = lidar_in.load().clone();
                let ctrl = self.control(&lidardata);
                {
                    let mut mtx = ctrl_out.write().unwrap();
					*mtx=ctrl;
                }
				thread::sleep(Duration::from_secs_f32(0.02));
            }
        })
    }
}
