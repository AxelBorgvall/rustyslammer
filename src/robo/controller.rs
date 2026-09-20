use crate::robo::{ImuState, LidarScan, Map};
use std::{collections::HashMap, sync::{Arc, RwLock, atomic::AtomicBool}, thread};
use arc_swap::{ArcSwap};


pub trait Controller: Send {
    fn spawn(&self, lidar_in:ArcSwap<LidarScan>);
}

pub struct BasicController {
	pub speed:f32,
	pub angvel:f32,
	pub reactrange:f32,
	pub k:f32,
}

impl BasicController{
	
}
