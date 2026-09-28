use crate::robo::{Chunk, ImuState, LidarScan, Map, MapQuery, SlamImage};
use arc_swap::ArcSwap;
use std::{
    collections::HashMap,
    sync::{Arc, RwLock, atomic::AtomicBool},
    thread::{self, JoinHandle},
};
use rand::thread_rng;
use rand_distr::{Normal,Distribution};
use std::time::Instant;
use std::thread::sleep;
use std::time::Duration;
/* --------------------------------- Config --------------------------------- */

pub trait Slam: Send {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        imu_in: Arc<RwLock<ImuState>>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_out: Arc<ArcSwap<Map>>,
		img_out: Option<Arc<ArcSwap<SlamImage>>>,
    ) -> JoinHandle<()>;
}

pub struct GMapping {
    // Params
    pub dx: f32,
    pub resampling_temp: f32,
    pub n_part: usize,
    pub ang_noise: f32,
    pub vel_noise: f32,
    pub search_distance: i32, // Radius searched around a lidar hit for terrain
    pub l_free: f32,
    pub l_occ: f32,

    // Scanmatch search params
    pub xy_step: f32,
    pub n_xy_steps: i32,
    pub th_step: f32,
    pub n_th_steps: i32,

    // State
    pub weights: Vec<f32>,
    pub particles: Vec<ImuState>,
    pub particle_maps: Vec<Map>,
    pub last_imu: ImuState,
	pub last_update:f32,
	
	// BUffers
	pub lp_buf:Vec<f32>,
	pub query_buf:Vec<MapQuery>,



}

impl GMapping {
    pub fn new(n_part: usize, dx: f32) -> Self {
        Self {
            dx: dx,
            resampling_temp: 8.0,
            n_part: n_part,
            ang_noise: 0.05,
            vel_noise: 0.05,
            search_distance: 3,
            l_free: 0.4,
            l_occ: 0.9,
            xy_step: dx,
            n_xy_steps: 2,
            th_step: 0.05,
            n_th_steps: 2,

            // Init these all to origin
            particles: vec![ImuState::default(); n_part],
            particle_maps: vec![Map::default();n_part],
            weights: vec![1.0; n_part],
            last_imu: ImuState::default(),
			last_update:0.0,
			
			lp_buf:vec![0.0;n_part],
			query_buf:vec![MapQuery::default();n_part],
        }
    }
	
	fn update_positions(&mut self,imu_data:ImuState,lidar_data:LidarScan,dt:f32){
		let delta=(imu_data-self.last_imu);
		let noise=(delta/dt).abs()*ImuState{
			x:self.vel_noise,y:self.vel_noise,theta:self.ang_noise,
		};

		self.last_imu=imu_data;
		
		let mut rng= thread_rng();
		let priors=self.particles.iter().zip(other)
		
		
	}
	
}

impl Slam for GMapping {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        imu_in: Arc<RwLock<ImuState>>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_out: Arc<ArcSwap<Map>>,
		img_out: Option<Arc<ArcSwap<SlamImage>>>,
    ) -> JoinHandle<()> {
        thread::spawn(move || {
            // Basic Slam loop goes here
        })
    }
	
	
}
