mod basic_sim;
mod sim_car;

use std::{sync::{Arc, RwLock, atomic::AtomicBool}, thread::JoinHandle};

use arc_swap::ArcSwap;
pub use basic_sim::*;
pub use sim_car::*;

use crate::robo::{EnvImage, LidarScan, RobotPose, TwoWheelControl};

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

