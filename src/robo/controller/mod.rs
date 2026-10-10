mod two_wheel_ctrl;
mod kbrd_ctrl;
mod hybrid_astar;

use std::{sync::{Arc, RwLock, atomic::AtomicBool}, thread::JoinHandle};

use arc_swap::ArcSwap;
pub use two_wheel_ctrl::*;
pub use kbrd_ctrl::*;
pub use hybrid_astar::*;

use crate::robo::{LidarScan, Map, RobotPose, SlamImage, TwoDOFControl};

pub trait Controller: Send {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_in: Arc<ArcSwap<Map>>,
		pos_in:Arc<RwLock<RobotPose>>,
        ctrl_out: Arc<RwLock<TwoDOFControl>>,
		img_out:  Option<Arc<ArcSwap<SlamImage>>>,
    ) -> JoinHandle<()>;
}
