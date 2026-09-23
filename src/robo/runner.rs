use std::{sync::{
    Arc, RwLock,
    atomic::{AtomicBool, Ordering::Relaxed},
}, time::Duration};

use arc_swap::ArcSwap;
use minifb::{Key, Window, WindowOptions};

use crate::robo::{
    EnvImage, ImuState, LidarScan, Map, SlamImage, TwoWheelControl, controller::Controller,
    environment::Environment, slam::Slam,
};

/* ----------------------------- Rendering stuff ---------------------------- */
const PANEL_SIZE: usize = 900;
const WINDOW_WIDTH: usize = PANEL_SIZE * 2;
const WINDOW_HEIGHT: usize = PANEL_SIZE;
const BG_DARK: u32 = 0x323232;
const BG_LIGHT: u32 = 0x7F7F7F;

fn fit_to_canvas(
    src_data: &[u32],
    src_w: usize,
    src_h: usize,
    max_w: usize,
    max_h: usize,
    bg_color: u32,
) -> Vec<u32> {
	// Return and empty frame if the image is 0size
    if src_w == 0 || src_h == 0 {
        return vec![bg_color; max_w * max_h];
    }

    let scale = f64::min(max_w as f64 / src_w as f64, max_h as f64 / src_h as f64);
    let new_w = (src_w as f64 * scale) as usize;
    let new_h = (src_h as f64 * scale) as usize;

    let pad_left = (max_w - new_w) / 2;
    let pad_top = (max_h - new_h) / 2;

    let mut out = vec![bg_color; max_w * max_h];

    for dst_y in 0..new_h {
        for dst_x in 0..new_w {
            let src_x = (dst_x as f64 / scale) as usize;
            let src_y = (dst_y as f64 / scale) as usize;

            let src_x = src_x.min(src_w.saturating_sub(1));
            let src_y = src_y.min(src_h.saturating_sub(1));

            let out_idx = (dst_y + pad_top) * max_w + (dst_x + pad_left);
            let src_idx = src_y * src_w + src_x;

            out[out_idx] = src_data[src_idx];
        }
    }
    out
}

/* -------------------------- Runner implementation ------------------------- */
struct SimRunner<E: Environment, S: Slam, C: Controller> {
    environment: E,
    slam: S,
    controller: C,
}

impl<E, S, C> SimRunner<E, S, C>
where
    E: Environment,
    S: Slam,
    C: Controller,
{
    pub fn new(environment: E, slam: S, controller: C) -> Self {
        Self {
            environment,
            slam,
            controller,
        }
    }

    pub fn start(&self) {
        // Set up mailboxes
        let mut shutdown_flag: Arc<AtomicBool> = Arc::new(AtomicBool::new(false));
        let mut imu_mailbox: Arc<RwLock<ImuState>> = Arc::new(RwLock::new(ImuState::default()));
        let mut lidarscan_mailbox: Arc<ArcSwap<LidarScan>> =
            Arc::new(ArcSwap::new(Arc::new(LidarScan::default())));
        let mut map_mailbox: Arc<ArcSwap<Map>> = Arc::new(ArcSwap::new(Arc::new(Map::new())));
        let mut control_mailbox: Arc<RwLock<TwoWheelControl>> =
            Arc::new(RwLock::new(TwoWheelControl::default()));

        let env_render_mailbox = Arc::new(ArcSwap::new(Arc::new(EnvImage::default())));
        let slam_render_mailbox = Arc::new(ArcSwap::new(Arc::new(SlamImage::default())));

        // launch threads
        let slam_handle = self.slam.spawn(
            shutdown_flag.clone(),
            imu_mailbox.clone(),
            lidarscan_mailbox.clone(),
            map_mailbox.clone(),
            Option::Some(slam_render_mailbox.clone()),
        );
        let env_handle = self.environment.spawn(
            shutdown_flag.clone(),
            lidarscan_mailbox.clone(),
            control_mailbox.clone(),
            Option::Some(env_render_mailbox.clone()),
        );
        let ctrl_handle = self.controller.spawn(
            shutdown_flag.clone(),
            lidarscan_mailbox.clone(),
            map_mailbox.clone(),
        );

        // Set up rendering
        let mut window = Window::new(
            "Robot SLAM - Press ESC or Q to exit",
            WINDOW_WIDTH,
            WINDOW_HEIGHT,
            WindowOptions::default(),
        )
        .expect("Failed to create window");
        window.limit_update_rate(Some(Duration::from_micros(50_000)));
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

            // 5. Handle exit conditions
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
