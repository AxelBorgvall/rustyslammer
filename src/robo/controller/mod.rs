mod two_wheel_ctrl;
mod kbrd_ctrl;

use std::{sync::{Arc, RwLock, atomic::AtomicBool}, thread::JoinHandle};

use arc_swap::ArcSwap;
pub use two_wheel_ctrl::*;
pub use kbrd_ctrl::*;

use crate::robo::{LidarScan, Map, TwoDOFControl};

pub trait Controller: Send {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_in: Arc<ArcSwap<Map>>,
        ctrl_out: Arc<RwLock<TwoDOFControl>>,
    ) -> JoinHandle<()>;
}
