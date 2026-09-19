# Program structure

Base the program on 4 main structs
- Slam: the actual slam logic
- Environment: either a simulated env or logic responsible for reading real lidar and IMU data
- Controller: logic for moving the robot, returns r_vals for speed and steering radius or somesuch
- Runner: Orchestrator struct holding one of each

Each struct will have its own trait with all it's required funcitonality so they can easily be swapped out. 

## SLAM
Does slam. Spins constantly and has multiple threads (for parallelized loops, rayon type stuff). Also has a rendering function that'll probably mostly be used in simulated mode. 

## Environment
It'll be called with something like get_state() and will give up integrated IMU readings + the latest complete lidar scan. This should be a separate thread that trips on some sort of signal in order for it to be able to do lidar readings in the background in the real usecase. For the simulation it'll probably be a thread sleeping until the data is requested. 

### Simulation
Step the simulation according to an internal dt param. Yield lidar scan + real odometry. When we want to expand and make more realistic we can have it spin while the SLAM happens so we have to deal with adjusting speed to fit slam iteration time and hadnling lidar being async to slam.

### Real car
We'll see when we get here. 

## controller
This should have access to both the slams tate, and lidar + imu data for creating a control solution. We're gonna want to design it in such a way that it can spin on its own thread in case we want to run some heave-ish pathfinding alg on the slam map in the future. 

## Runner
Sets up shared data channels and joiins threads when they quit.

# data flow
## Shutdown state
Any individual struct should be able to trigger a fatal error. Keep an Arc<AtomicBool> in the orcehstrator and pass to each new process.  they simply check ```while !shutdown_flag.load(Ordering::Relaxed)```. If any struct hits a fatal error, it calls ```shutdown_flag.store(true, Ordering::Relaxed)``` and breaks its own loop. The other threads will see the true flag on their next iteration and exit cleanly. The Runner, holding the thread JoinHandles, waits for them all to finish and safely terminates the program.
### IMU
IMU data is tiny. Let orchestrator make a shared ```Arc<RwLock<ImuState>>``` and pass into all processes. When someone wants the data they copy it. Give internal IMU struct the clone+copy trait and we can use:

```
// copy
let current: Odometry = *odom.read().unwrap();

//write
*odom.write().unwrap() = new_odom;
```

## Lidar
Lidar is one writer many reader. The controller creates an ```ArcSwap<LidarScan>``` which is passed to each substruct. 

1. Data is always shared thourgh the ```ArcSwap<LidarScan>```
2. Env streams scan into internal buffer. 
3. When the buffer is full, the ENV wraps it in an arc and calls swap() onthe arcswap. 
4. WHen slam or controller wants to read they ask the arcswap for a snapshot and get an arc pointing to teh data. 

No one block the env writing. Env and controller can grab completely async

## Slam map
Slam map has to be exposed to the controller. Otherwise teh slam is kinda useless. The individual maps are already ```HashMap<(i32,i32),Arc[f32;CHUNK_SIZE]``` so we only need to expose the outer hashmap of the best particle. The make_mut function used when updating the maps will always ensure that the data chunks held by another particle or handed of to controller will never be overwritten and instead copied.  

```
// Update map chunk
let chunk_arc = particle_map.entry((chunk_x, chunk_y))
    .or_insert_with(|| Arc::new([0.0; CHUNK_SIZE])); // Create new if unseen

// Handle COW and in-place mutation
let chunk_data = Arc::make_mut(chunk_arc);

// Now mutate `chunk_data` (&mut [f32; CHUNK_SIZE])
chunk_data[cell_index] += log_odds_update;
```

```
// resampling is done via a shallow cloen()
let new_particle_map = surviving_particle_map.clone();
```

```
// Passing to controller like:
let best_map_snapshot = best_particle_hashmap.clone();

// Has to be mapped in an arc in order to cross thread boundaries. 
controller_mailbox.store(Arc::new(best_map_snapshot));
```

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
odometry+scan pair odometry is change in pose and scan an array of lidar distances+angles

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






# Prompt followup. 
1. I like this. We can have some sort of shutdowntime flag that any struct is allowed to trip if it encounters a fatal error and then the controller can kill all the processes. What si teh nicest way to do something like that. 
2. The idea here sounds good also. 
3. I think we had a misscommunication here. "However, you mentioned you want to pass "non-owning slices" to threads." I emant that the SLAM main thread would have a main owning datastructure for the slam data but in the readonly parallell rayon loops we could easily pass nonowning slices with complete certainty that they will not outlive the owning structrue. That seems like something that is probably possible right? Even if it technically is passing non owned data into subthreads it is obvious the threads will be shorter lived that the data. Does rayon have some sort of loop written with an unsafe block that will do this or do we just wrap the data in an arc and forget about ti? I guess that rwlock solution might still be real good and flexible. How should env handle the writing? We want env to constantly spin lidar data, probably into an internal buffer, then shoot that into the buffer whenver it becomes free to write into. Also this would necessitate the use of some "stale scan" atomic flag that slam can use to signal that its done with the data. If that flag is up controller should also keep its hands off the data so that slam (slowest heaviest process (probably)) can get a new one. 
4. The controller grabbing lidar+imu the same way as slam 100% makes sense. What specific functionality is best suited for slam publishing a slamstate do you think? We dont want the controller and slam to block each others reads or writes since they might both end up fairly heavy when the controller starts doing pathfinding and stuff. The planned map format for the slam is a hashmap from i32,i32 index tuples to contiguous data chunks. 

# Your questions
1. to start with teh controller will be much faster, but we will be interested in making it more complex later and it might do heavy math on the slam map. Due to this i think it will be best if the slam can share its data to teh controller without either ever blocking the other for very long. 
2. My first simulation will be completely synchronous for simplicity but i do want to make an asynchronous one since it will be more realistic. Any design decisions related to the data transfer should presume an asynchronous simulation so we dont trip over our old choices. 
3. I'm guessing just teh best map? Is a weighted sum too heavy as well do you think? I dont really know whats suitable for a gmapping slam so am open to input here. 


# prompt followup followwup