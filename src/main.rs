use crate::robo::{controller::BasicController, environment::SimEnv, slam::OGMapping,runner::SimRunner};

mod robo;
fn main() {
	// env_logger::init();
    let env = SimEnv::new("data/map1.dat");
    let controller = BasicController::default();
	let slammer=OGMapping::new(30, 0.05);
	let runner=SimRunner::new(env, slammer, controller);
	
	runner.start();
}
