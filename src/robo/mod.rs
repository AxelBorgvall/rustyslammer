pub mod controller;
pub mod environment;
pub mod slam;
pub mod runner;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use arc_swap::ArcSwap;




/* ------------------------------- Define map ------------------------------- */
const CHUNK_L: usize = 32;
const CHUNK_SIZE: usize = CHUNK_L * CHUNK_L;
pub type Chunk=[f32;CHUNK_SIZE];
pub type Map=HashMap<(i32,i32),Arc<Chunk>>;

/* --------------------------- Define lidar state --------------------------- */
#[derive(Clone)]
struct LidarScan {
	pub ranges:Vec<f32>,
	pub angles:Vec<f32>,
	pub max_distance: f32,
}

/* ----------------------------- Define ImuState ---------------------------- */
#[derive(Clone,Copy)]
struct ImuState {
	x:f32,
	y:f32,
	theta:f32,
}
impl Default for ImuState{
	fn default() -> Self {
		Self {
			x:0.0,y:0.0,theta:0.0
		}
	}
}

/* --------------------------- define Controlinput -------------------------- */
#[derive(Clone,Copy)]
struct TwoWheelControl{
	v_r:f32,
	om_r:f32,
}