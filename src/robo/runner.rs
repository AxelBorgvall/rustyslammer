use crate::robo::{
	environment::Environment,
	slam::Slam,
	controller::Controller,
};


struct SimRunner<E:Environment,S:Slam,C:Controller> {
	environment:E,
	slam:S,
	controller:C,
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
	
	pub fn start(){
		
	}
}




