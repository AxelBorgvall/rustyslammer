pub mod bresenham;
pub mod controller;
pub mod environment;
pub mod io;
pub mod runner;
pub mod slam;

// use std::collections::HashMap;
use rustc_hash::FxHashMap;
use std::ops::{Add, Div, Mul, Sub};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use arc_swap::ArcSwap;

/* ------------------------------- Define map ------------------------------- */
pub struct Cell {
	pub hits:u16,
	pub visits:u16,
	pub mx:f32, // Mean hit position: [0,1)
	pub my:f32,
}
impl Cell {
    #[inline]
    pub fn occupied(&self, thresh: f32) -> bool {
        self.hits > 0 && self.hits as f32 >= thresh * self.visits as f32
    }
    pub fn occupancy(&self) -> f32 {
        if self.visits == 0 { 0.5 } else { self.hits as f32 / self.visits as f32 }
    }
}
const CHUNK_L: usize = 32;
const CHUNK_SIZE: usize = CHUNK_L * CHUNK_L;
pub type Chunk = [Cell; CHUNK_SIZE];
pub type Map = FxHashMap<(i32, i32), Arc<Chunk>>;

#[derive(Debug,Clone)]
pub struct MapQuery {
    chunk_coord: (i32, i32),
    localx: u16,
    localy: u16,
    prop_id: u32,
}
impl Default for MapQuery {
    fn default() -> Self {
        Self {
            chunk_coord: (0, 0),
            localx: 0,
            localy: 0,
            prop_id: 0,
        }
    }
}

/* --------------------------- Define lidar state --------------------------- */
#[derive(Clone)]
pub struct LidarScan {
    pub ranges: Vec<f32>,
    pub angles: Vec<f32>,
    pub max_distance: f32,
}
impl Default for LidarScan {
    fn default() -> Self {
        Self {
            ranges: vec![],
            angles: vec![],
            max_distance: 0.0,
        }
    }
}

/* ----------------------------- Define ImuState ---------------------------- */
#[derive(Clone, Copy)]
pub struct ImuState {
    x: f32,
    y: f32,
    theta: f32,
}
impl Default for ImuState {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }
}
impl Add for ImuState {
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            theta: self.theta + other.theta,
        }
    }
}

impl Sub for ImuState {
    type Output = Self;
    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            theta: self.theta - other.theta,
        }
    }
}
impl Mul for ImuState {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            theta: self.theta * rhs.theta,
        }
    }
}
impl Div<f32> for ImuState{
	type Output =Self;
	fn div(self, rhs: f32) -> Self::Output {
		Self{
			x:self.x/rhs,
			y:self.y/rhs,
			theta:self.theta/rhs,
		}
		
	} 
}
impl  ImuState{
	fn abs(self)->Self{
		Self{
			x:self.x.abs(),
			y:self.y.abs(),
			theta:self.theta.abs(),
			
		}
	}

}

/* --------------------------- define Controlinput -------------------------- */
#[derive(Clone, Copy, Debug)]
pub struct TwoWheelControl {
    v_r: f32,
    om_r: f32,
}
impl Default for TwoWheelControl {
    fn default() -> Self {
        Self {
            v_r: 0.0,
            om_r: 0.0,
        }
    }
}

/* ------------------------- define screen/rendering ------------------------ */

pub struct EnvImage {
    height: usize,
    width: usize,
    data: Vec<u32>,
}

impl Default for EnvImage {
    fn default() -> Self {
        Self {
            height: 0,
            width: 0,
            data: vec![],
        }
    }
}

pub struct SlamImage {
    height: usize,
    width: usize,
    data: Vec<u32>,
}

impl Default for SlamImage {
    fn default() -> Self {
        Self {
            height: 0,
            width: 0,
            data: vec![],
        }
    }
}
