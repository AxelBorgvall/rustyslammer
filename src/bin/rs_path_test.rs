use std::f32::consts::PI;

use minifb::{Key, Window, WindowOptions};
use rustyslammer::{
    EnvImage, RobotPose, SlamImage, environment::CarEnv, reeds_shepp::get_optimal_path, runner::{BG_LIGHT, WINDOW_HEIGHT, WINDOW_WIDTH, fit_to_canvas},
};
const dx: f32 = 0.05;

fn draw_reeds_shepp(img:&mut EnvImage,start:RobotPose,end:RobotPose,turning_rad:f32){
	get_optimal_path(start, end)
}

fn main() {
    let mut env = CarEnv::new("data/openroom.dat");
    let mut window = Window::new(
        "Reeds Shepp test",
        WINDOW_HEIGHT,
        WINDOW_HEIGHT,
        WindowOptions::default(),
    )
    .expect("failed to create window");
    window.set_target_fps(20);
    let mut combined_buffer = vec![0u32; WINDOW_HEIGHT * WINDOW_HEIGHT];
	
	let nh=env.nh;
	let nw=env.nw;
	let mut endpose=env.pose;
	endpose.theta=(endpose.theta*1.5)%2.*PI;


    while window.is_open() {
        env.render(None);
        let mut env_buffer = env.screenbuffer.clone();
		

		let frame=fit_to_canvas(&env_buffer, nw, nh, WINDOW_HEIGHT, WINDOW_HEIGHT, BG_LIGHT);
		window.update_with_buffer(&frame, WINDOW_HEIGHT, WINDOW_HEIGHT);
		if window.is_key_down(Key::Escape)||window.is_key_down(Key::Q){
			break;
		}
    }
}
