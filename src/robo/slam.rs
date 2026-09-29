use crate::robo::{Chunk, ImuState, LidarScan, Map, MapQuery, SlamImage};
use arc_swap::ArcSwap;
use std::{
    collections::HashMap, sync::{Arc, RwLock, atomic::{AtomicBool, Ordering::Relaxed}}, thread::{self, JoinHandle},
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

pub struct OGMapping {
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
    pub max_steps: i32,
    pub th_step: f32,
    pub max_halvings: i32,

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

impl OGMapping {
    pub fn new(n_part: usize, dx: f32) -> Self {
        Self {
            dx: dx,
            resampling_temp: 8.0,
            n_part: n_part,
            ang_noise: 0.05,
            vel_noise: 0.05,
            search_distance: 1,
            l_free: 0.4,
            l_occ: 0.9,
            xy_step: dx,
            max_steps: 60,
            th_step: 0.05,
            max_halvings: 4,

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
	
	fn compute_dist(&self,prior:ImuState,imu_data:ImuState,lidar_data:&LidarScan){

	}
	pub fn update_positions(&mut self,imu_data:ImuState,lidar_data:&LidarScan,dt:f32){
		// Compute priors
		let delta=(imu_data-self.last_imu);
		self.last_imu=imu_data;
		
		let mut rng= thread_rng();
		let nosie_dist_x=Normal::new(0.0,(delta.x/dt*self.vel_noise).abs()+0.005).unwrap();
		let nosie_dist_y=Normal::new(0.0,(delta.y/dt*self.vel_noise).abs()+0.005).unwrap();
		let nosie_dist_theta=Normal::new(0.0,(delta.theta/dt*self.ang_noise).abs()+0.01).unwrap();
		
		let priors:Vec<ImuState>=self.particles.iter().map(|particle|{
			ImuState {
				x:particle.x+nosie_dist_x.sample(&mut rng),
				y:particle.y+nosie_dist_y.sample(&mut rng),
				theta:particle.theta+nosie_dist_theta.sample(&mut rng),
			}
		}).collect();
		let sigsqr:f32=(0.2f32).powi(2);
		
		for prior in priors{
			self.compute_dist(prior, imu_data, &lidar_data);
		}
		
	}
	
}

impl Slam for OGMapping {
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
			while !shutdown_flag.load(Relaxed){

			}
        })
    }
	
	
}
