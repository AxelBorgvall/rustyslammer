# Program structure

Base the program on 4 main structs
- Slam: the actual slam logic
- Environment: either a simulated env or logic responsible for reading real lidar and IMU data
- Controller: logic for moving the robot, returns r_vals for speed and steering radius or somesuch
- Runner: Orchestrator struct holding one of each

Each struct will have its own trait with all it's required funcitonality so they can easily be swapped out. 

## SLAM
Does slam. Spins constantly and has multiple threads. Also has a rendering function that'll probably mostly be used in simulated mode. 

## Environment
It'll be called with something like get_state() and will give up integrated IMU readings + the latest complete lidar scan. This should be a separate thread that trips on some sort of signal in order for it to be able to do lidar readings in the background in the real usecase. For the simulation it'll be a thread sleeping until the data is requested. 

### Simulation
Step the simulation according to an internal dt param. Yield lidar scan + real odometry.
### Real car
Keep a running "latest complete lidar scan" in a buffer. Replacing when a new one is avaliable. Grab Integrated IMU readings from MCU and Yield buffer whenever the function is called. There can be two lidar buffers, one for streaming data into and one with a mutex that can be written to by the env. 


## Controller
I want it to have access to lidar data, and a map. I'll just use lidar data + wander about for the simulation but in real life we'll want to have some proper nav logic, including the slam map. I dont know how the many particle maps should be represented. This will be called from the env when its time to make a control input to the real or simulated robot. Dont think itll need its own thread? But maybe if we are goind to be running A* and shit its just as well to set it up as a live thread that can spin concurrently to the slam? I'm really not sure. It can go ahead and get the best map. 

## Runner
I dont know if runner should be an orchestrator or just let the spinning slam thread grab data from env whenever it wants and flag controller to grab it and so on. 

## MCU
Will need to integrate and hand over IMU on request and take r values for outputs whenver they arrive. 

# GMAPPING algorithm
## Data

### bufs and stuffs
- mapquery 
```
{
	chunk_coord:(i32,i32),
	localx:u16,
	localy:u16,
	prop_id:u32, // flattened proposition id
}
```

- Particle poses: x,y,theta tuples for all particles
- Map: Every particle holds a hashmap from outer indices (i32,i32) to a nxn patch of map data. The patch is represented as Arc<[f32;256] by keeping them as refcounted objects they can be updated lazily via Rc::make_mut() only when particles no longer share that map segment. This evens out expensive mem operations. Use a fast hasher like rustc-hahs for the hashmap.
- pointbuffers: A buffer of the projected lidar points for all the different scanmatch samples every element will be a MapQuery[Nray,(searchRadius*2+1)^3] for each particle
- logprob_buffers: a buffer of all the running lop probs for the different sampled particles [Npart,(searchRadius*2+1)^3] give each particle each own logprob buffer
- 

## Input
odometry+scan pair odometry is change in pose and scan an array of lidar distances

## Kinematics
- for all particles modify them by the odometry with an added noise model do drive them in different ways. Store in priors
## Scan match
The scan match scales as particles x rays x scanmatch_rad^3 x searchspace^2 and all of it is trivial operations, so completely memory limited. Computing distances by chunk rather than by particle sample will be critical. 

- compute all projected points into the particlewise mapquery buffers (wrap pi for the pose samples)
- ```sort_usntable_by_key(|q| q.chunk_coord)``` unstable sort over chunk coords
- Do: 
```
for chunk_group in queries.chunk_by(|a, b| a.chunk_coord == b.chunk_coord) {
    
    let current_chunk_coord = chunk_group[0].chunk_coord;
    let chunk_data = map.get_chunk(current_chunk_coord);
    
    for q in chunk_group {
        let dist = chunk_data.get_distance(q.local_x, q.local_y);
        let likelihood = compute_likelihood(dist);
        log_probs[q.prop_id as usize] += likelihood;
    }
}
```
- weigths and covariance are computed from the lp so a running tally instead of raywise distances is fine

## state update
- pull new poses from the computed distributions (make sure to wrap pi!)
- update weights
### map update
I'm not sure between
1. Store all updates into a vector buffer then apply chunkwise
2. Just trace each ray one at a time







