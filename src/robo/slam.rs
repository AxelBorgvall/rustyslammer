use crate::robo::{ImuState, LidarScan, Map};
use arc_swap::ArcSwap;
use std::{
    collections::HashMap,
    sync::{Arc, RwLock, atomic::AtomicBool},
    thread::{self, JoinHandle},
};

/* --------------------------------- Config --------------------------------- */

pub trait Slam: Send {
    fn spawn(
        &self,
        shutdown_flag: Arc<AtomicBool>,
        imu_in: Arc<RwLock<ImuState>>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_out: Arc<ArcSwap<Map>>,
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
            particle_maps: vec![],
            weights: vec![1.0; n_part],
            last_imu: ImuState::default(),
        }
    }
}

impl Slam for GMapping {
    fn spawn(
        &self,
        shutdown_flag: Arc<AtomicBool>,
        imu_in: Arc<RwLock<ImuState>>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_out: Arc<ArcSwap<Map>>,
    ) -> JoinHandle<()> {
        thread::spawn(move || {
            // Basic Slam loop goes here
        })
    }
}
