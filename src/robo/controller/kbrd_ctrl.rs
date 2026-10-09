use crate::robo::{LidarScan, Map, TwoDOFControl, controller::Controller};
use arc_swap::ArcSwap;
use device_query::{DeviceQuery, DeviceState, Keycode};
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

impl Controller for KeyBoardController {
    fn spawn(
        self,
        shutdown_flag: Arc<AtomicBool>,
        lidar_in: Arc<ArcSwap<LidarScan>>,
        map_in: Arc<ArcSwap<Map>>,
        ctrl_out: Arc<RwLock<TwoDOFControl>>,
    ) -> JoinHandle<()> {
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
                if keys.contains(&Keycode::A) || keys.contains(&Keycode::Left) {
                    whang += self.max_whang;
                }
                if keys.contains(&Keycode::D) || keys.contains(&Keycode::Right) {
                    whang -= self.max_whang;
                }

                {
                    let mut mtx = ctrl_out.write().unwrap();
                    mtx.trans_r = speed;
                    mtx.rot_r = whang;
                }

                thread::sleep(Duration::from_secs_f32(0.02));
            }
        })
    }
}
