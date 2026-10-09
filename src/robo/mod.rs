#![allow(unused)]
pub mod bresenham;
pub mod controller;
pub mod environment;
pub mod io;
pub mod runner;
pub mod slam;

// use std::collections::HashMap;
use rustc_hash::FxHashMap;
use std::ops::{Add, Div, Mul, Sub};
use std::sync::{Arc,};

/* ------------------------------- Define map ------------------------------- */
const COUNT_CAP: u16 = u16::MAX / 2;
#[derive(Clone, Copy, Default)]
pub struct Cell {
    pub hits: u16,
    pub visits: u16,
    pub mx: f32, // Mean hit position: [0,1)
    pub my: f32,
}
impl Cell {
    #[inline]
    pub fn occupied(&self, thresh: f32) -> bool {
        self.hits > 0 && self.hits as f32 >= thresh * self.visits as f32
    }
    pub fn occupancy(&self) -> f32 {
        if self.visits == 0 {
            0.5
        } else {
            self.hits as f32 / self.visits as f32
        }
    }
    pub fn add_free(&mut self) {
        if self.visits >= COUNT_CAP {
            self.visits /= 2;
            self.hits /= 2;
        }
        self.visits += 1;
    }
    pub fn add_hit(&mut self, cx: f32, cy: f32) {
        self.add_free();
        self.hits += 1;
        let n = self.hits as f32;
        self.mx += (cx - self.mx) / n;
        self.my += (cy - self.my) / n;
    }
}
pub fn world2cell(x: f32, y: f32, dx: f32) -> (i32, i32) {
    ((x / dx).floor() as i32, (y / dx).floor() as i32)
}

const CHUNK_L: usize = 32;
const CHUNK_SIZE: usize = CHUNK_L * CHUNK_L;
pub type Chunk = [Cell; CHUNK_SIZE];
pub type Map = FxHashMap<(i32, i32), Arc<Chunk>>;

pub struct ChunkCache<const N: usize> {
    keys: [(i32, i32); N],
    ptrs: [*mut Chunk; N],
    next: usize,
}
impl<const N: usize> ChunkCache<N> {
    fn new() -> Self {
        Self {
            keys: [(i32::MAX, i32::MAX); N],
            ptrs: [std::ptr::null_mut(); N],
            next: 0,
        }
    }
    #[inline]
    fn get_mut(&mut self, map: &mut Map, key: (i32, i32)) -> *mut Chunk {
        for i in 0..N {
            if self.keys[i] == key {
                return self.ptrs[i];
            }
        }
        // Chunk not in cache
        let chunk = map
            .entry(key)
            .or_insert_with(|| Arc::new([Cell::default(); CHUNK_SIZE]));
        let p = Arc::make_mut(chunk) as *mut _;
        self.ptrs[self.next] = p;
        self.keys[self.next] = key;
        self.next = (self.next + 1) % N;
        p
    }

    #[inline]
    fn get(&mut self, map: &mut Map, key: (i32, i32)) -> *const Chunk {
        for i in 0..N {
            if self.keys[i] == key {
                return self.ptrs[i];
            }
        }
        // Chunk not in cache
        let chunk = map
            .entry(key)
            .or_insert_with(|| Arc::new([Cell::default(); CHUNK_SIZE]));
        let p = Arc::make_mut(chunk) as *mut _;
        self.ptrs[self.next] = p;
        self.keys[self.next] = key;
        self.next = (self.next + 1) % N;
        p
    }

    #[inline]
    fn add_free(&mut self, map: &mut Map, pos: (i32, i32)) {
        const L: i32 = CHUNK_L as i32;
        let chunk_key = (pos.0.div_euclid(L), pos.1.div_euclid(L));
        let local_x = pos.0.rem_euclid(L) as usize;
        let local_y = pos.1.rem_euclid(L) as usize;
        unsafe {
            (*self.get_mut(map, chunk_key))[local_x + local_y * CHUNK_L].add_free();
        }
    }

    #[inline]
    fn add_hit(&mut self, map: &mut Map, idx_pos: (i32, i32), real_pos: (f32, f32), dx: f32) {
        const L: i32 = CHUNK_L as i32;
        let chunk_key = (idx_pos.0.div_euclid(L), idx_pos.1.div_euclid(L));
        let local_x = idx_pos.0.rem_euclid(L) as usize;
        let local_y = idx_pos.1.rem_euclid(L) as usize;
        unsafe {
            (*self.get_mut(map, chunk_key))[local_x + local_y * CHUNK_L].add_hit(
                (real_pos.0 / dx) - (idx_pos.0 as f32),
                (real_pos.1 / dx) - (idx_pos.1 as f32),
            );
        }
    }
}

fn _cell_mut(map: &mut Map, wx: i32, wy: i32) -> &mut Cell {
    const L: i32 = CHUNK_L as i32;
    let chunk = map
        .entry((wx.div_euclid(L), wy.div_euclid(L)))
        .or_insert_with(|| Arc::new([Cell::default(); CHUNK_SIZE]));
    let cells = Arc::make_mut(chunk);
    &mut cells[wy.rem_euclid(L) as usize * CHUNK_L + wx.rem_euclid(L) as usize]
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
pub struct ScanPoints {
    pub hits: Vec<[f32; 2]>,
    pub free_only: Vec<[f32; 2]>,
}

impl ScanPoints {
    pub fn from_scan(scan: &LidarScan, max_range: f32) -> Self {
        let mut hits = Vec::with_capacity(scan.ranges.len());
        let mut free_only = Vec::new();
        for (&r, &a) in scan.ranges.iter().zip(&scan.angles) {
            if !r.is_finite() {
                continue;
            }
            let (s, c) = a.sin_cos();
            if r < max_range {
                hits.push([r * c, r * s]);
            } else {
                free_only.push([max_range * c, max_range * s]);
            }
        }
        Self { hits, free_only }
    }
}

pub fn to_world<'a>(pts: &'a [[f32; 2]], pose: RobotPose) -> impl Iterator<Item = [f32; 2]> + 'a {
    let (s, c) = pose.theta.sin_cos();
    let (px, py) = (pose.x, pose.y);
    pts.iter()
        .map(move |&[x, y]| [px + c * x - s * y, py + s * x + c * y])
}

// Private structs for my stupid beam major point iterator
#[derive(Clone, Copy)]
struct PoseTransform {
    x: f32,
    y: f32,
    s: f32,
    c: f32,
}
impl PoseTransform {
    fn new(pose: RobotPose) -> Self {
        let (s, c) = pose.theta.sin_cos();
        Self {
            x: pose.x,
            y: pose.y,
            s,
            c,
        }
    }
    #[inline]
    fn transform(self, [x, y]: [f32; 2]) -> [f32; 2] {
        [
            self.x + self.c * x - self.s * y,
            self.y + self.s * x + self.c * y,
        ]
    }
}
struct BeamMajor<'a> {
    pts: &'a [[f32; 2]],
    poses: Vec<PoseTransform>,
    beam_idx: usize,
    pose_idx: usize,
}
impl<'a> Iterator for BeamMajor<'a> {
    type Item = (usize, [f32; 2]);
    fn next(&mut self) -> Option<Self::Item> {
        if self.pts.is_empty() || self.poses.is_empty() {
            return None;
        }

        if self.beam_idx >= self.pts.len() {
            return None;
        }
        let pt = self.pts[self.beam_idx];
        let pose_idx = self.pose_idx;
        let world = self.poses[pose_idx].transform(pt);
        self.pose_idx += 1;
        if self.pose_idx == self.poses.len() {
            self.pose_idx = 0;
            self.beam_idx += 1;
        }

        Some((pose_idx, world))
    }
}
pub fn to_world_beam_major<'a>(
	pts: &'a [[f32; 2]],
	poses:&[RobotPose],
)->impl Iterator<Item = (usize,[f32;2])>+'a{
	let pose_trans=poses.iter().copied().map(PoseTransform::new).collect();
	BeamMajor {
		pts,
		poses:pose_trans,
		beam_idx:0,
		pose_idx:0,
	}
}

/* ----------------------------- Define ImuState ---------------------------- */
#[derive(Clone, Copy)]
pub struct RobotPose {
    x: f32,
    y: f32,
    theta: f32,
}
impl Default for RobotPose {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }
}
impl Add for RobotPose {
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            theta: self.theta + other.theta,
        }
    }
}

impl Sub for RobotPose {
    type Output = Self;
    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            theta: self.theta - other.theta,
        }
    }
}
impl Mul for RobotPose {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            theta: self.theta * rhs.theta,
        }
    }
}
impl Div<f32> for RobotPose {
    type Output = Self;
    fn div(self, rhs: f32) -> Self::Output {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
            theta: self.theta / rhs,
        }
    }
}
impl RobotPose {
    fn abs(self) -> Self {
        Self {
            x: self.x.abs(),
            y: self.y.abs(),
            theta: self.theta.abs(),
        }
    }
	fn from_pos(p:Pos,theta:f32)->Self{
		Self { x: p.x, y: p.y, theta }
	}
}
#[derive(Clone, Copy)] // I literally just want numerical syntax
pub struct Pos{
	x:f32,
	y:f32,
}
impl Add for Pos {
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Pos {
    type Output = Self;
    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}
impl Mul for Pos {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
        }
    }
}
impl Mul<f32> for Pos {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Pos{
	fn astup(self)->(f32,f32){
		(self.x,self.y)
	}
}

/* --------------------------- define Controlinput -------------------------- */
#[derive(Clone, Copy, Debug)]
pub struct TwoDOFControl {
    trans_r: f32,
    rot_r: f32,
}
impl Default for TwoDOFControl {
    fn default() -> Self {
        Self {
            trans_r: 0.0,
            rot_r: 0.0,
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
