use std::{
    sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering::Relaxed},
    },
};

use arc_swap::ArcSwap;
use minifb::{Key, Window, WindowOptions};

use crate::robo::{
    EnvImage, LidarScan, Map, RobotPose, SlamImage, TwoDOFControl, controller::Controller, environment::Environment, runner::{BG_DARK, BG_LIGHT, PANEL_SIZE, WINDOW_HEIGHT, WINDOW_WIDTH, fit_to_canvas}, slam::Slam,
};

/* -------------------------- Runner implementation ------------------------- */
pub struct SimRunner<E: Environment, S: Slam, C: Controller> {
    environment: E,
    slam: S,
    controller: C,
}

impl<E, S, C> SimRunner<E, S, C>
where
    E: Environment + Send + 'static,
    S: Slam + Send + 'static,
    C: Controller + Send + 'static,
{
    pub fn new(environment: E, slam: S, controller: C) -> Self {
        Self {
            environment,
            slam,
            controller,
        }
    }

    pub fn start(self) {
        // Set up mailboxes
        let shutdown_flag: Arc<AtomicBool> = Arc::new(AtomicBool::new(false));
        let imu_mailbox: Arc<RwLock<RobotPose>> = Arc::new(RwLock::new(RobotPose{x:f32::NAN,y:f32::NAN, theta:f32::NAN}));
        let lidarscan_mailbox: Arc<ArcSwap<LidarScan>> =
            Arc::new(ArcSwap::new(Arc::new(LidarScan::default())));
        let map_mailbox: Arc<ArcSwap<Map>> = Arc::new(ArcSwap::new(Arc::new(Map::default())));
        let control_mailbox: Arc<RwLock<TwoDOFControl>> =
            Arc::new(RwLock::new(TwoDOFControl::default()));
        let pos_mailbox: Arc<RwLock<RobotPose>> = Arc::new(RwLock::new(RobotPose{x:f32::NAN,y:f32::NAN, theta:f32::NAN}));

        let env_render_mailbox = Arc::new(ArcSwap::new(Arc::new(EnvImage::default())));
        let slam_render_mailbox = Arc::new(ArcSwap::new(Arc::new(SlamImage::default())));

        // launch threads
        let slam_handle = self.slam.spawn(
            shutdown_flag.clone(),
            imu_mailbox.clone(),
            lidarscan_mailbox.clone(),
			pos_mailbox.clone(),
            map_mailbox.clone(),
            Option::Some(slam_render_mailbox.clone()),
        );
        let env_handle = self.environment.spawn(
            shutdown_flag.clone(),
            lidarscan_mailbox.clone(),
            imu_mailbox.clone(),
            control_mailbox.clone(),
            Option::Some(env_render_mailbox.clone()),
        );
        let ctrl_handle = self.controller.spawn(
            shutdown_flag.clone(),
            lidarscan_mailbox.clone(),
            map_mailbox.clone(),
			pos_mailbox.clone(),
            control_mailbox.clone(),
			None,
        );

        // Set up rendering
        let mut window = Window::new(
            "Robot SLAM - Press ESC or Q to exit",
            WINDOW_WIDTH,
            WINDOW_HEIGHT,
            WindowOptions::default(),
        )
        .expect("Failed to create window");
        window.set_target_fps(20);
        let mut combined_buffer = vec![0u32; WINDOW_WIDTH * WINDOW_HEIGHT];

        while !shutdown_flag.load(Relaxed) && window.is_open() {
            let env_img = env_render_mailbox.load();
            let slam_img = slam_render_mailbox.load();

            let frame_left = fit_to_canvas(
                &env_img.data,
                env_img.width,
                env_img.height,
                PANEL_SIZE,
                PANEL_SIZE,
                BG_DARK,
            );

            let frame_right = fit_to_canvas(
                &slam_img.data,
                slam_img.width,
                slam_img.height,
                PANEL_SIZE,
                PANEL_SIZE,
                BG_LIGHT,
            );

            // Concatenate horizontally
            for y in 0..PANEL_SIZE {
                let left_start = y * PANEL_SIZE;
                let left_end = left_start + PANEL_SIZE;

                let combined_start = y * WINDOW_WIDTH;
                let combined_mid = combined_start + PANEL_SIZE;
                let combined_end = combined_start + WINDOW_WIDTH;

                combined_buffer[combined_start..combined_mid]
                    .copy_from_slice(&frame_left[left_start..left_end]);

                combined_buffer[combined_mid..combined_end]
                    .copy_from_slice(&frame_right[left_start..left_end]);
            }

            // Draw to screen
            window
                .update_with_buffer(&combined_buffer, WINDOW_WIDTH, WINDOW_HEIGHT)
                .unwrap();

			// Exit con
            if window.is_key_down(Key::Escape) || window.is_key_down(Key::Q) {
                shutdown_flag.store(true, Relaxed);
            }
        }
        shutdown_flag.store(true, Relaxed);

        slam_handle.join().expect("SLAM thread panicked!");
        env_handle.join().expect("Environment thread panicked!");
        ctrl_handle.join().expect("Controller thread panicked!");
    }
}
