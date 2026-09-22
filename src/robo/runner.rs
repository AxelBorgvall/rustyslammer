use std::sync::{Arc, RwLock, atomic::AtomicBool};

use arc_swap::ArcSwap;

use crate::robo::{
    ImuState, LidarScan, Map, TwoWheelControl, controller::Controller, environment::Environment,
    slam::Slam,
};

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
        let mut shutdown_flag: Arc<AtomicBool> = Arc::new(AtomicBool::new(false));
        let mut imu_mailbox: Arc<RwLock<ImuState>> = Arc::new(RwLock::new(ImuState::default()));
        let mut lidarscan_mailbox: Arc<ArcSwap<LidarScan>> =
            Arc::new(ArcSwap::new(Arc::new(LidarScan::default())));
        let mut map_mailbox: Arc<ArcSwap<Map>> = Arc::new(ArcSwap::new(Arc::new(Map::new())));
        let mut control_mailbox: Arc<RwLock<TwoWheelControl>> =
            Arc::new(RwLock::new(TwoWheelControl::default()));

        let slam_handle = self.slam.spawn(
            shutdown_flag.clone(),
            imu_mailbox.clone(),
            lidarscan_mailbox.clone(),
            map_mailbox.clone(),
        );
        let env_handle = self.environment.spawn(
            shutdown_flag.clone(),
            lidarscan_mailbox.clone(),
            control_mailbox.clone(),
        );
        let ctrl_handle = self.controller.spawn(
            shutdown_flag.clone(),
            lidarscan_mailbox.clone(),
            map_mailbox.clone(),
        );

        slam_handle.join().expect("SLAM thread panicked!");
        env_handle.join().expect("Environment thread panicked!");
        ctrl_handle.join().expect("Controller thread panicked!");
    }
}
