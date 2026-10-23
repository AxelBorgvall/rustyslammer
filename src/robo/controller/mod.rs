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
use rustc_hash::FxHashMap;
pub use two_wheel_ctrl::*;

use crate::robo::{LidarScan, Map, RobotPose, SlamImage, TwoDOFControl};

pub const CHUNK_L: usize = 32;
pub const CHUNK_SIZE: usize = CHUNK_L * CHUNK_L;
pub type NumChunk = [f32; CHUNK_SIZE];
pub type NMap = FxHashMap<(i32, i32), NumChunk>;

pub struct RSMap{
	pub l:usize,	
	pub n_ang:usize,
	pub dx:f32,	
	pub offset:(f32,f32),
	pub dtheta:f32,
	pub cost:Vec<f32>,
}
impl RSMap{
	pub fn new(dx:f32,dtheta:f32,goal:RobotPose,width:f32)->Self{
		todo!();
	}
	pub fn get(&self,pose:RobotPose)->&f32{
		let x=((pose.x-self.offset.0)/self.dx) as usize;
		let y=((pose.y-self.offset.1)/self.dx) as usize;
		let theta=(pose.theta/self.dtheta) as usize;
		&self.cost[y*self.l*self.n_ang+x*self.n_ang+theta]
	}
	pub fn get_mut(&mut self,pose:RobotPose)->&mut f32{
		let x=((pose.x-self.offset.0)/self.dx) as usize;
		let y=((pose.y-self.offset.1)/self.dx) as usize;
		let theta=(pose.theta/self.dtheta) as usize;
		&mut self.cost[y*self.l*self.n_ang+x*self.n_ang+theta]
	}
}

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
