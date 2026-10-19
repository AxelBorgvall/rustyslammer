use std::{f32::consts::PI };

use minifb::{Key, Window, WindowOptions};
use rustyslammer::{
    RobotPose, bresenham::Bresenham, environment::CarEnv, reeds_shepp::{Gear, Steer, optimal_reeds_shepp}, runner::{BG_LIGHT, WINDOW_HEIGHT, fit_to_canvas}, util::{ draw_path},
};

const GREEN: u32 = 0x0000FF00;
const BLACK: u32 = 0x00000000;
const FORWARD_COLOR: u32 = 0x00FF0000;
const REVERSE_COLOR: u32 = 0x000000FF;

fn put(buf: &mut [u32], w: usize, h: usize, x: i32, y: i32, color: u32) {
    if x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h {
        buf[x as usize + y as usize * w] = color;
    }
}

fn to_px(x: f32, y: f32, dx: f32) -> (i32, i32) {
    ((x / dx).floor() as i32, (y / dx).floor() as i32)
}

fn draw_reeds_shepp(
    buf: &mut Vec<u32>,
    h: usize,
    w: usize,
	dx:f32,
    start: RobotPose,
    end: RobotPose,
    turning_rad: f32,
) {
	// Draw endpose
    let (ex, ey) = to_px(end.x, end.y,dx);
    for j in ey - 5..ey + 5 {
        for i in ex - 5..ex + 5 {
            put(buf, w, h, i, j, GREEN);
        }
    }
    let nose_tip = (
        (end.x / dx + 6. * end.theta.cos()).round() as i32,
        (end.y / dx + 6. * end.theta.sin()).round() as i32,
    );
    for (i, j) in Bresenham::new((ex, ey), nose_tip) {
        put(buf, w, h, i, j, BLACK);
    }

    let Some(path) = optimal_reeds_shepp(start, end, turning_rad) else {
        return;
    };
	
	draw_path(&path, &mut *buf, h, w, dx, start, turning_rad, FORWARD_COLOR, REVERSE_COLOR);

	// Check position
    let r = turning_rad;
    let mut pose = start;

    for seg in path {
        if seg.param <= 1e-6 {
            continue;
        }

        // Signed distance travelled along the path: negative when reversing
        let s = match seg.gear {
            Gear::Forward => seg.param,
            Gear::Reverse => -seg.param,
        };

        match seg.steer {
            Steer::Straight => {
                let next = RobotPose {
                    x: pose.x + pose.theta.cos() * s,
                    y: pose.y + pose.theta.sin() * s,
                    theta: pose.theta,
                };
                pose = next;
            }
            Steer::Left | Steer::Right => {
                // k = +1 for a left turn, -1 for right
                let k = if matches!(seg.steer, Steer::Left) { 1.0 } else { -1.0 };
                let (sin_t, cos_t) = pose.theta.sin_cos();

                let cx = pose.x - k * sin_t * r;
                let cy = pose.y + k * cos_t * r;

                let dtheta = k * s / r;
                let a0 = pose.theta - k * PI / 2.;
                let a1 = a0 + dtheta;

                pose = RobotPose {
                    x: cx + r * a1.cos(),
                    y: cy + r * a1.sin(),
                    theta: pose.theta + dtheta,
                };
            }
        }
    }

	// Print if we miss the goal
    let pos_err = (pose.x - end.x).hypot(pose.y - end.y);
    let dth = pose.theta - end.theta;
    let ang_err = dth.sin().atan2(dth.cos()).abs();
    if pos_err > 1e-2 || ang_err > 1e-2 {
        eprintln!("RS path misses goal: pos err {pos_err:.4} m, angle err {ang_err:.4} rad");
    }
}

fn main() {
    let mut env = CarEnv::new("data/openroom.dat");
	env.max_whang/=5.;
    let mut window = Window::new(
        "Reeds Shepp test",
        WINDOW_HEIGHT,
        WINDOW_HEIGHT,
        WindowOptions::default(),
    )
    .expect("failed to create window");
    window.set_target_fps(30);

    let nh = env.nh;
    let nw = env.nw;
    let mut endpose = env.pose;
    let turn_rad = env.wh_l / env.max_whang.tan();
    endpose.theta = (endpose.theta * 1.5).rem_euclid(2. * PI) ;

    while window.is_open() {
        env.render(None);
        let mut env_buffer = env.screenbuffer.clone();
        draw_reeds_shepp(&mut env_buffer, nh, nw, env.dx,env.pose, endpose, turn_rad);
        let frame = fit_to_canvas(&env_buffer, nw, nh, WINDOW_HEIGHT, WINDOW_HEIGHT, BG_LIGHT);
        window
            .update_with_buffer(&frame, WINDOW_HEIGHT, WINDOW_HEIGHT)
            .unwrap();

        if window.is_key_down(Key::W) {
            endpose.y -= 0.1f32
        }
        if window.is_key_down(Key::S) {
            endpose.y += 0.1f32;
        }
        if window.is_key_down(Key::A) {
            endpose.x -= 0.1f32;
        }
        if window.is_key_down(Key::D) {
            endpose.x += 0.1f32;
        }
        if window.is_key_down(Key::E) {
            endpose.theta -= 0.1;
        }
        if window.is_key_down(Key::Q) {
            endpose.theta += 0.1;
        }
        if window.is_key_down(Key::Escape) {
            break;
        }
    }
}
