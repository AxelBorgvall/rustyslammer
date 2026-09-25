use crate::robo::{ImuState, LidarScan, Map, TwoWheelControl};
use arc_swap::ArcSwap;
use std::{
    collections::HashMap,
    f32::consts::PI,
    sync::{Arc, RwLock, atomic::AtomicBool},
    thread::{self, JoinHandle},
};

pub trait Controller: Send {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_in: Arc<ArcSwap<Map>>,
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
            speed: 0.2,
            angvel: 0.2,
            reactrange: 2.0,
            k: 1.0,
        }
    }
}

impl BasicController {
    fn control(&self, scan: &LidarScan) -> TwoWheelControl {
        let mut fx = 0.0f32;
        let mut fy = 0.0f32;

        let nrays_f = scan.angles.len() as f32;
        let max_dist = scan.max_distance - 0.1;

        for (&angle, &range) in scan.angles.iter().zip(scan.ranges.iter()) {
            if range > max_dist || angle.abs() > PI {
                continue;
            }

            let mag = self.k / (range.powi(2) + 0.01);
            fx += mag * angle.cos();
            fy += mag * angle.sin();
        }
        if nrays_f > 0.0 {
            fx /= nrays_f;
            fy /= nrays_f;
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
    ) -> JoinHandle<()> {
        thread::spawn(move || {
            // Basic Slam loop goes here
        })
    }
}
