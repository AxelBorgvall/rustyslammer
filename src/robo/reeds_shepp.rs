use std::f32::consts::PI;

use arrayvec::ArrayVec;

use crate::robo::RobotPose;

// Util

pub fn angle_norm(theta: f32) -> f32 {
    let mut theta = theta.rem_euclid(2. * PI);
    if theta >= PI {
        theta -= 2. * PI
    } else if theta < -PI {
        theta += 2. * PI
    }
    theta
}

pub struct Polar {
    pub rho: f32,
    pub theta: f32,
}

pub fn cartesian_to_polar(x: f32, y: f32) -> Polar {
    let rho = x.hypot(y);
    let theta = y.atan2(x);
    Polar { rho, theta }
}

pub fn change_of_basis(p1: RobotPose, p2: RobotPose) -> RobotPose {
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;

    let cos_theta1 = p1.theta.cos();
    let sin_theta1 = p2.theta.sin();

    let x_new = dx * cos_theta1 + dy * sin_theta1;
    let y_new = -dx * sin_theta1 + dy * cos_theta1;

    let theta_new = p2.theta - p1.theta;

    RobotPose {
        x: x_new,
        y: y_new,
        theta: theta_new,
    }
}



#[derive(Debug,Clone,Copy,PartialEq)]
pub enum Steer {
    Left,
    Right,
    Straight,
}
impl Steer {
	pub fn reverse(&self)->Self{
		match self {
			Steer::Left=>Steer::Right,
			Steer::Right=>Steer::Left,
			Steer::Straight=>Steer::Straight,
		}
	}
}

#[derive(Debug,Clone,Copy,PartialEq,Eq)]
pub enum Gear {
    Forward,
    Reverse,
}
impl Gear {
	pub fn reverse(&self)->Self {
		match self {
			Gear::Forward=>Gear::Reverse,
			Gear::Reverse=>Gear::Forward,
		}
	}
}

#[derive(Debug,Clone,Copy)]
pub struct Segment {
	pub steer:Steer,
	pub gear:Gear,
	pub param:f32,
}
impl Segment {
	fn new(param:f32,steer:Steer,gear:Gear)->Self{
		if param>=0.{
			Segment { steer, gear, param }
		} else {
			Segment { steer, gear:gear.reverse(), param:-param }
		}
	}
	fn reverse_steer(&mut self){
		self.steer=self.steer.reverse();
	}
	fn reverse_gear(&mut self){
		self.gear=self.gear.reverse();
	}
}

pub type RSPath=ArrayVec<Segment,5>;


pub fn timeflip(path: RSPath) -> RSPath {
    path.into_iter()
        .map(|mut e| {
            e.reverse_gear();
            e
        })
        .collect()
}

pub fn reflect(path: RSPath) -> RSPath {
    path.into_iter()
        .map(|mut e| {
            e.reverse_steer();
            e
        })
        .collect()
}

pub fn path_length(path: &RSPath) -> f32 {
    path.iter().map(|e| e.param.abs()).sum()
}
pub type PathFn=fn(f32,f32,f32)->RSPath;
pub const PATH_FNS:[PathFn;12]=[
	
    path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12,
];

pub fn get_optimal_path(start:RobotPose,end:RobotPose)->Option<RSPath>{
	None
}

pub fn get_all_paths(start:RobotPose,end:RobotPose)-> ArrayVec<RSPath,46>{
	ArrayVec::new()
}
