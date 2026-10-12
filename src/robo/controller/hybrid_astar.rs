use crate::robo::{
    LidarScan, Map, RobotPose, SlamImage, TwoDOFControl,
    controller::{CarPath, Controller, PathFollower},
};
use arc_swap::ArcSwap;
use device_query::{DeviceQuery, DeviceState, Keycode};
use std::{
    f32::consts::PI,
    sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering::Relaxed},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

struct HybridAStarCore {}
impl HybridAStarCore {
    fn new() -> Self {
        Self {}
    }
}

pub struct HybridAStar {
	dijk_heur:Map,
	last_map:Option<Arc<Map>>
}

impl HybridAStar {
    fn new() -> Self {
        Self {
			dijk_heur:Map::default(),
			last_map:None,
		}
    }
    pub fn render(&self, map: &Map) {}
}

impl Controller for HybridAStar {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_in: Arc<ArcSwap<Map>>,
        pos_in: Arc<RwLock<RobotPose>>,
        ctrl_out: Arc<RwLock<TwoDOFControl>>,
        img_out: Option<Arc<ArcSwap<SlamImage>>>,
    ) -> JoinHandle<()> {
        thread::spawn(move || {
            let mut path_mbox = Arc::new(ArcSwap::new(Arc::new(CarPath::new())));

            // PathFollower loop
            let path_follow_handle = {
                // Shadowing arc clones for the follower loop
                let shutdown_flag = shutdown_flag.clone();
                let pos_in = pos_in.clone();
                let path_mbox = path_mbox.clone();

                let mut lastcall = Instant::now();
                thread::spawn(move || {
                    println!("PathFollower thread starting now");
                    while !shutdown_flag.load(Relaxed) {
                        // Publish control out
                        let now = Instant::now();
                        let dt = now - lastcall;
                        lastcall = now;
                    }
                })
            };

            // HybridAStar loop
            println!("HybridAStar thread starting now");
            let mut lastcall = Instant::now();
            let mut lastdraw = Instant::now();
            while !shutdown_flag.load(Relaxed) {
                let now = Instant::now();
                let dt = now - lastcall;
                lastcall = now;
                // Publish environment render
            }
            path_follow_handle
                .join()
                .expect("PathFollower thread panicked!");
        })
    }
}
