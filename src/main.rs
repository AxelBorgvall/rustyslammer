use crate::robo::{
    controller::{BasicController, KeyBoardController}, environment::{BasicSimEnv, CarEnv}, runner::SimRunner, slam::OGMapping,
};

mod robo;
// fn main() {
//     let env = BasicSimEnv::new("data/map1.dat");
//     let controller = BasicController::default();
//     let slammer = OGMapping::new(30, 0.05);
//     let runner = SimRunner::new(env, slammer, controller);

//     runner.start();
// }

fn main() {
    let env = CarEnv::new("data/map1.dat");
    let controller = KeyBoardController::default();
    let slammer = OGMapping::new(30, 0.05);
    let runner = SimRunner::new(env, slammer, controller);

    runner.start();
}