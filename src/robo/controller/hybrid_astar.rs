use crate::robo::{LidarScan, Map, RobotPose, SlamImage, TwoDOFControl, controller::Controller};
use arc_swap::ArcSwap;
use device_query::{DeviceQuery, DeviceState, Keycode};
use std::{
    f32::consts::PI, sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering::Relaxed},
    }, thread::{self, JoinHandle}, time::{Duration, Instant},
};

pub struct HybridAStar {
}

impl HybridAStar {
	fn new()->Self{
		Self {

		}
	}
	pub fn render(&self,map:&Map){
		
	}
}

impl Controller for HybridAStar {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_in: Arc<ArcSwap<Map>>,
		pos_in:Arc<RwLock<RobotPose>>,
        ctrl_out: Arc<RwLock<TwoDOFControl>>,
		img_out:  Option<Arc<ArcSwap<SlamImage>>>,
    ) -> JoinHandle<()> {
		assert!(img_out.is_none(),"KeyBoardController does not draw images, pass None instead");
        thread::spawn(move || {
            let device_state = DeviceState::new();
            println!("Environment thread starting now");
            let mut lastcall = Instant::now();
            let mut lastdraw = Instant::now();
            while !shutdown_flag.load(Relaxed) {
				
                let now = Instant::now();
                let dt = now - lastcall;
                lastcall = now;
                // Publish environment render
            }
        })
    }
}
