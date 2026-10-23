use crate::{controller::NMap, robo::{
    LidarScan, Map, RobotPose, SlamImage, TwoDOFControl,
    controller::{CarPath, Controller, PathFollower},
}};
use arc_swap::ArcSwap;
use std::{
    f32::consts::PI,
    sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering::Relaxed},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

#[derive(Debug,Clone, Copy,PartialEq)]
struct State {
	pose:RobotPose,
	f_score:f32,
	cost:f32,
}
impl Eq for State {}
impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for State {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .f_score
            .total_cmp(&self.f_score)
            .then_with(|| self.cost.total_cmp(&other.cost))
    }
}

struct HybridAStarCore {
	dx_map:f32, // Incoming map dx
	dx_state:f32, // Internal state quantization
	dth_state:f32,
	rs_heur_range:f32,
}
impl HybridAStarCore {
    fn new() -> Self {
        Self {
			dx_map:0.05,
			dx_state:0.1,
			dth_state:0.1,
			rs_heur_range:100.
		}
    }
	fn compute_dijkstra_heur(&self,map:Map)->NMap{
		let cost=NMap::default();
		todo!("Implement dijk");
		cost
	}
	fn recompute_reeds_shepp(&self)->NMap{
		let cost=NMap::default();
		todo!("Implement rs");
		cost
	}
}

pub struct HybridAStar {
	dijk_heur:NMap,
	rs_heur:NMap,
	last_map:Option<Arc<Map>>
}

impl HybridAStar {
    fn new(rs_heur_path:&str) -> Self {
		
        Self {
			dijk_heur:NMap::default(),
			rs_heur:NMap::default(),
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
