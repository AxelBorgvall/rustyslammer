mod hybrid_astar;
mod kbrd_ctrl;
mod two_wheel_ctrl;

use std::{
    collections::VecDeque,
    sync::{Arc, RwLock, atomic::AtomicBool},
    thread::JoinHandle,
};

use arc_swap::ArcSwap;
pub use hybrid_astar::*;
pub use kbrd_ctrl::*;
use rayon::collections::vec_deque;
pub use two_wheel_ctrl::*;

use crate::robo::{LidarScan, Map, RobotPose, SlamImage, TwoDOFControl};

pub trait Controller: Send {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_in: Arc<ArcSwap<Map>>,
        pos_in: Arc<RwLock<RobotPose>>,
        ctrl_out: Arc<RwLock<TwoDOFControl>>,
        img_out: Option<Arc<ArcSwap<SlamImage>>>,
    ) -> JoinHandle<()>;
}

pub struct CarPathPoint {
    pose: RobotPose,
    fwd: bool,
}

pub struct CarPath {
    pub poses: VecDeque<CarPathPoint>,
}
impl CarPath {
    pub fn new() -> Self {
        Self {
            poses: VecDeque::new(),
        }
    }
}
pub struct PathFollower {}

impl PathFollower {}
