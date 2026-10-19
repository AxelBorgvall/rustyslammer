use std::{f32::consts::PI, path};

use arrayvec::ArrayVec;

use crate::robo::RobotPose;

// Public interface
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Steer {
    Left,
    Right,
    Straight,
}
impl Steer {
    pub fn reverse(&self) -> Self {
        match self {
            Steer::Left => Steer::Right,
            Steer::Right => Steer::Left,
            Steer::Straight => Steer::Straight,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Gear {
    Forward,
    Reverse,
}
impl Gear {
    pub fn reverse(&self) -> Self {
        match self {
            Gear::Forward => Gear::Reverse,
            Gear::Reverse => Gear::Forward,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Segment {
    pub steer: Steer,
    pub gear: Gear,
    pub param: f32,
}
impl Segment {
    fn new(param: f32, steer: Steer, gear: Gear) -> Self {
        if param >= 0. {
            Segment { steer, gear, param }
        } else {
            Segment {
                steer,
                gear: gear.reverse(),
                param: -param,
            }
        }
    }
    fn reverse_steer(&mut self) {
        self.steer = self.steer.reverse();
    }
    fn reverse_gear(&mut self) {
        self.gear = self.gear.reverse();
    }
}

pub type RSPath = ArrayVec<Segment, 5>;

pub fn path_length(path: &RSPath) -> f32 {
    path.iter().map(|e| e.param.abs()).sum()
}

pub fn optimal_reeds_shepp(
    start: RobotPose,
    end: RobotPose,
    turning_radius: f32,
) -> Option<RSPath> {
    let norm_start = RobotPose {
        x: start.x / turning_radius,
        y: start.y / turning_radius,
        theta: start.theta,
    };
    let norm_end = RobotPose {
        x: end.x / turning_radius,
        y: end.y / turning_radius,
        theta: end.theta,
    };

    match get_optimal_path(norm_start, norm_end) {
        Some(mut path) => {
            unnorm_path(&mut path, turning_radius);
            Some(path)
        }
        None => None,
    }
}
pub fn all_reeds_shepp(
    start: RobotPose,
    end: RobotPose,
    turning_radius: f32,
) -> ArrayVec<RSPath, 48> {
    let norm_start = RobotPose {
        x: start.x / turning_radius,
        y: start.y / turning_radius,
        theta: start.theta,
    };
    let norm_end = RobotPose {
        x: end.x / turning_radius,
        y: end.y / turning_radius,
        theta: end.theta,
    };
    let mut paths = get_all_paths(norm_start, norm_end);
    paths
        .iter_mut()
        .for_each(|path| unnorm_path(path, turning_radius));
    paths
}

// Private functions
pub fn unnorm_path(path: &mut RSPath, r: f32) {
    path.iter_mut().for_each(|segment| segment.param *= r);
}
pub fn norm_path(path:&mut RSPath,r:f32){
	let fac=1./r;
    path.iter_mut().for_each(|segment| segment.param *= fac);
}
fn angle_norm(theta: f32) -> f32 {
    let mut theta = theta.rem_euclid(2. * PI);
    if theta >= PI {
        theta -= 2. * PI
    }
    theta
}

struct Polar {
    pub rho: f32,
    pub theta: f32,
}

fn cartesian_to_polar(x: f32, y: f32) -> Polar {
    let rho = x.hypot(y);
    let theta = y.atan2(x);
    Polar { rho, theta }
}

fn change_of_basis(p1: RobotPose, p2: RobotPose) -> RobotPose {
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;

    let cos_theta1 = p1.theta.cos();
    let sin_theta1 = p1.theta.sin();

    let x_new = dx * cos_theta1 + dy * sin_theta1;
    let y_new = -dx * sin_theta1 + dy * cos_theta1;
    let theta_new = angle_norm(p2.theta - p1.theta);

    RobotPose {
        x: x_new,
        y: y_new,
        theta: theta_new,
    }
}

fn timeflip(path: RSPath) -> RSPath {
    path.into_iter()
        .map(|mut e| {
            e.reverse_gear();
            e
        })
        .collect()
}

fn reflect(path: RSPath) -> RSPath {
    path.into_iter()
        .map(|mut e| {
            e.reverse_steer();
            e
        })
        .collect()
}

fn path_param_length(path: &RSPath) -> f32 {
    path.iter().map(|e| e.param.abs()).sum()
}
type PathFn = fn(f32, f32, f32) -> RSPath;
const PATH_FNS: [PathFn; 12] = [
    path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12,
];

fn get_optimal_path(start: RobotPose, end: RobotPose) -> Option<RSPath> {
    iter_paths(start, end).min_by(|a, b| {
        let la = path_param_length(a);
        let lb = path_param_length(b);
        la.is_nan()
            .cmp(&lb.is_nan())
            .then_with(|| la.total_cmp(&lb))
    })
}
fn iter_paths(start: RobotPose, end: RobotPose) -> impl Iterator<Item = RSPath> {
    let pose = change_of_basis(start, end);
    let (x, y, theta) = (pose.x, pose.y, pose.theta);
    PATH_FNS
        .into_iter()
        .flat_map(move |get_path| {
            [
                get_path(x, y, theta),
                timeflip(get_path(-x, y, -theta)),
                reflect(get_path(x, -y, -theta)),
                reflect(timeflip(get_path(-x, -y, theta))),
            ]
        })
        .filter(|p| !p.is_empty())
}
fn get_all_paths(start: RobotPose, end: RobotPose) -> ArrayVec<RSPath, 48> {
    iter_paths(start, end).collect()
}

fn path1(x: f32, y: f32, phi: f32) -> RSPath {
    let polar = cartesian_to_polar(x - phi.sin(), y - 1. + phi.cos());
    let v = angle_norm(phi - polar.theta);

    [
        Segment::new(polar.theta, Steer::Left, Gear::Forward),
        Segment::new(polar.rho, Steer::Straight, Gear::Forward),
        Segment::new(v, Steer::Left, Gear::Forward),
    ]
    .into_iter()
    .collect()
}

fn path2(x: f32, y: f32, phi: f32) -> RSPath {
    let polar = cartesian_to_polar(x + phi.sin(), y - 1. - phi.cos());

    let rho = polar.rho;
    let theta = polar.theta;

    if rho * rho >= 4. {
        let u = (rho * rho - 4.).sqrt();
        let t = angle_norm(theta + (2.0_f32).atan2(u));
        let v = angle_norm(t - phi);

        [
            Segment::new(t, Steer::Left, Gear::Forward),
            Segment::new(u, Steer::Straight, Gear::Forward),
            Segment::new(v, Steer::Right, Gear::Forward),
        ]
        .into_iter()
        .collect()
    } else {
        ArrayVec::new()
    }
}

fn path3(x: f32, y: f32, phi: f32) -> RSPath {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let polar = cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if polar.rho <= 4. {
        let a = (rho / 4.).acos();
        let t = angle_norm(theta + PI / 2. + a);
        let u = angle_norm(PI - 2. * a);
        let v = angle_norm(phi - t - u);

        [
            Segment::new(t, Steer::Left, Gear::Forward),
            Segment::new(u, Steer::Right, Gear::Reverse),
            Segment::new(v, Steer::Left, Gear::Forward),
        ]
        .into_iter()
        .collect()
    } else {
        ArrayVec::new()
    }
}

fn path4(x: f32, y: f32, phi: f32) -> RSPath {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let polar = cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho <= 4. {
        let a = (rho / 4.).acos();
        let t = angle_norm(theta + PI / 2. + a);
        let u = angle_norm(PI - 2. * a);
        let v = angle_norm(t + u - phi);

        [
            Segment::new(t, Steer::Left, Gear::Forward),
            Segment::new(u, Steer::Right, Gear::Reverse),
            Segment::new(v, Steer::Left, Gear::Reverse),
        ]
        .into_iter()
        .collect()
    } else {
        ArrayVec::new()
    }
}

fn path5(x: f32, y: f32, phi: f32) -> RSPath {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let polar = cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho <= 4. {
        let u = (1. - rho * rho / 8.).acos();
        let a = (2. * u.sin() / rho).clamp(-1.0, 1.0).asin();
        let t = angle_norm(theta + PI / 2. - a);
        let v = angle_norm(t - u - phi);

        [
            Segment::new(t, Steer::Left, Gear::Forward),
            Segment::new(u, Steer::Right, Gear::Forward),
            Segment::new(v, Steer::Left, Gear::Reverse),
        ]
        .into_iter()
        .collect()
    } else {
        ArrayVec::new()
    }
}

fn path6(x: f32, y: f32, phi: f32) -> RSPath {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let polar = cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho <= 4. {
        let (t, u, v);
        if rho <= 2. {
            let a = ((rho + 2.) / 4.).acos();
            t = angle_norm(theta + PI / 2. + a);
            u = angle_norm(a);
            v = angle_norm(phi - t + 2. * u);
        } else {
            let a = ((rho - 2.) / 4.).acos();
            t = angle_norm(theta + PI / 2. - a);
            u = angle_norm(PI - a);
            v = angle_norm(phi - t + 2. * u);
        }

        [
            Segment::new(t, Steer::Left, Gear::Forward),
            Segment::new(u, Steer::Right, Gear::Forward),
            Segment::new(u, Steer::Left, Gear::Reverse),
            Segment::new(v, Steer::Right, Gear::Reverse),
        ]
        .into_iter()
        .collect()
    } else {
        ArrayVec::new()
    }
}

fn path7(x: f32, y: f32, phi: f32) -> RSPath {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let polar = cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    let u1 = (20. - rho * rho) / 16.;

    if rho <= 6. && (0. ..=1.).contains(&u1) {
        let u = u1.acos();
        let a = (2. * u.sin() / rho).clamp(-1.0, 1.0).asin();
        let t = angle_norm(theta + PI / 2. + a);
        let v = angle_norm(t - phi);

        [
            Segment::new(t, Steer::Left, Gear::Forward),
            Segment::new(u, Steer::Right, Gear::Reverse),
            Segment::new(u, Steer::Left, Gear::Reverse),
            Segment::new(v, Steer::Right, Gear::Forward),
        ]
        .into_iter()
        .collect()
    } else {
        ArrayVec::new()
    }
}

fn path8(x: f32, y: f32, phi: f32) -> RSPath {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let polar = cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho >= 2. {
        let sqrt_arg = rho * rho - 4.;
        if sqrt_arg < 0. {
            return ArrayVec::new();
        }

        let s = sqrt_arg.sqrt();
        let u_param = s - 2.;
        if u_param < 0. {
            return ArrayVec::new();
        }

        let a = (2.0_f32).atan2(s);
        let t = angle_norm(theta + PI / 2. + a);
        let v = angle_norm(t - phi + PI / 2.);

        [
            Segment::new(t, Steer::Left, Gear::Forward),
            Segment::new(PI / 2., Steer::Right, Gear::Reverse),
            Segment::new(u_param, Steer::Straight, Gear::Reverse),
            Segment::new(v, Steer::Left, Gear::Reverse),
        ]
        .into_iter()
        .collect()
    } else {
        ArrayVec::new()
    }
}

fn path9(x: f32, y: f32, phi: f32) -> RSPath {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let polar = cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho >= 2. {
        let sqrt_arg = rho * rho - 4.;
        if sqrt_arg < 0. {
            return ArrayVec::new();
        }

        let s = sqrt_arg.sqrt();
        let u_param = s - 2.;
        if u_param < 0. {
            return ArrayVec::new();
        }

        let a = s.atan2(2.0_f32);
        let t = angle_norm(theta + PI / 2. - a);
        let v = angle_norm(t - phi - PI / 2.);

        [
            Segment::new(t, Steer::Left, Gear::Forward),
            Segment::new(u_param, Steer::Straight, Gear::Forward),
            Segment::new(PI / 2., Steer::Right, Gear::Forward),
            Segment::new(v, Steer::Left, Gear::Reverse),
        ]
        .into_iter()
        .collect()
    } else {
        ArrayVec::new()
    }
}

fn path10(x: f32, y: f32, phi: f32) -> RSPath {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let polar = cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho >= 2. {
        let t = angle_norm(theta + PI / 2.);
        let u = rho - 2.;
        let v = angle_norm(phi - t - PI / 2.);

        [
            Segment::new(t, Steer::Left, Gear::Forward),
            Segment::new(PI / 2., Steer::Right, Gear::Reverse),
            Segment::new(u, Steer::Straight, Gear::Reverse),
            Segment::new(v, Steer::Right, Gear::Reverse),
        ]
        .into_iter()
        .collect()
    } else {
        ArrayVec::new()
    }
}

fn path11(x: f32, y: f32, phi: f32) -> RSPath {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let polar = cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho >= 2. {
        let t = angle_norm(theta);
        let u = rho - 2.;
        let v = angle_norm(phi - t - PI / 2.);

        [
            Segment::new(t, Steer::Left, Gear::Forward),
            Segment::new(u, Steer::Straight, Gear::Forward),
            Segment::new(PI / 2., Steer::Left, Gear::Forward),
            Segment::new(v, Steer::Right, Gear::Reverse),
        ]
        .into_iter()
        .collect()
    } else {
        ArrayVec::new()
    }
}

fn path12(x: f32, y: f32, phi: f32) -> RSPath {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let polar = cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho >= 4. {
        let sqrt_base_arg = rho * rho - 4.;
        if sqrt_base_arg < 0. {
            return ArrayVec::new();
        }

        let u_base = sqrt_base_arg.sqrt();
        let u_param = u_base - 4.;

        if u_param < 0. {
            return ArrayVec::new();
        }
        let s_equiv = u_base;

        let a = (2.0_f32).atan2(s_equiv);
        let t = angle_norm(theta + PI / 2. + a);
        let v = angle_norm(t - phi);

        [
            Segment::new(t, Steer::Left, Gear::Forward),
            Segment::new(PI / 2., Steer::Right, Gear::Reverse),
            Segment::new(u_param, Steer::Straight, Gear::Reverse),
            Segment::new(PI / 2., Steer::Left, Gear::Reverse),
            Segment::new(v, Steer::Right, Gear::Forward),
        ]
        .into_iter()
        .collect()
    } else {
        ArrayVec::new()
    }
}
