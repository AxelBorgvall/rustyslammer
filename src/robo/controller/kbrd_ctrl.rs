use crate::robo::{LidarScan, Map, RobotPose, SlamImage, TwoDOFControl, controller::Controller};
use arc_swap::ArcSwap;
use std::{
    f32::consts::PI,
    sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering::Relaxed},
    },
    thread::{self, JoinHandle},
    time::Duration,
};

pub struct KeyBoardController {
    pub max_speed: f32,
    pub max_whang: f32,
}

impl Default for KeyBoardController {
    fn default() -> Self {
        Self {
            max_speed: 1.0,
            max_whang: 45.0 * PI / 180.0,
        }
    }
}

#[cfg(target_os = "windows")]
use device_query::{DeviceQuery, DeviceState, Keycode};

#[cfg(target_os = "windows")]
impl Controller for KeyBoardController {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_in: Arc<ArcSwap<Map>>,
        pos_in: Arc<RwLock<RobotPose>>,
        ctrl_out: Arc<RwLock<TwoDOFControl>>,
        img_out: Option<Arc<ArcSwap<SlamImage>>>,
    ) -> JoinHandle<()> {
        assert!(img_out.is_none(), "KeyBoardController does not draw images");

        thread::spawn(move || {
            let device_state = DeviceState::new();

            while !shutdown_flag.load(Relaxed) {
                let keys: Vec<Keycode> = device_state.get_keys();
                let mut speed = 0.0;
                let mut whang = 0.0;

                if keys.contains(&Keycode::W) || keys.contains(&Keycode::Up) {
                    speed += self.max_speed;
                }
                if keys.contains(&Keycode::S) || keys.contains(&Keycode::Down) {
                    speed -= self.max_speed;
                }
                if keys.contains(&Keycode::D) || keys.contains(&Keycode::Right) {
                    whang += self.max_whang;
                }
                if keys.contains(&Keycode::A) || keys.contains(&Keycode::Left) {
                    whang -= self.max_whang;
                }

                if let Ok(mut mtx) = ctrl_out.write() {
                    mtx.trans_r = speed;
                    mtx.rot_r = whang;
                }
                thread::sleep(Duration::from_secs_f32(0.01));
            }
        })
    }
}

#[cfg(target_os = "linux")]
use evdev::Key;

#[cfg(target_os = "linux")]
impl Controller for KeyBoardController {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_in: Arc<ArcSwap<Map>>,
        pos_in: Arc<RwLock<RobotPose>>,
        ctrl_out: Arc<RwLock<TwoDOFControl>>,
        img_out: Option<Arc<ArcSwap<SlamImage>>>,
    ) -> JoinHandle<()> {
        assert!(img_out.is_none(), "KeyBoardController does not draw images");

        thread::spawn(move || {
            // Find the first device that has a 'W' key (filters out mice/power buttons)
            let mut keyboard = evdev::enumerate()
                .find(|(_, dev)| {
                    dev.supported_keys()
                        .map_or(false, |keys| keys.contains(Key::KEY_W))
                })
                .map(|(_, dev)| dev)
                .expect("No keyboard found in /dev/input!");

            while !shutdown_flag.load(Relaxed) {
                let keys = keyboard.get_key_state().unwrap();

                let mut speed = 0.0;
                let mut whang = 0.0;

                if keys.contains(Key::KEY_W) || keys.contains(Key::KEY_UP) {
                    speed += self.max_speed;
                }
                if keys.contains(Key::KEY_S) || keys.contains(Key::KEY_DOWN) {
                    speed -= self.max_speed;
                }
                if keys.contains(Key::KEY_D) || keys.contains(Key::KEY_RIGHT) {
                    whang += self.max_whang;
                }
                if keys.contains(Key::KEY_A) || keys.contains(Key::KEY_LEFT) {
                    whang -= self.max_whang;
                }

                if let Ok(mut mtx) = ctrl_out.write() {
                    mtx.trans_r = speed;
                    mtx.rot_r = whang;
                }
                thread::sleep(Duration::from_secs_f32(0.01));
            }
        })
    }
}
