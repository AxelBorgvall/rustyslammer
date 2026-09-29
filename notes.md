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

For lookups use cell.div_euclid(32 as i32) for the chunk coord and cell.rem_euclid(32) for teh local index. 

Use rayon in outermost loop, over particles. 


## Position Update + Likelyhood (OG)
Create priors with noisy odometry 
### Hill climb
```
let mut best = start;
let mut best_score = score(&best);
let (mut lin, mut ang) = (self.xy_step, self.th_step);
let (mut halvings, mut moves) = (0, 0);

while halvings < MAX_HALVINGS && moves < MAX_MOVES {
    let cands = [
        ImuState { x: best.x + lin, ..best }, ImuState { x: best.x - lin, ..best },
        ImuState { y: best.y + lin, ..best }, ImuState { y: best.y - lin, ..best },
        ImuState { theta: best.theta + ang, ..best }, ImuState { theta: best.theta - ang, ..best },
    ];
    // f32 isn't Ord, so use total_cmp
    let (c, s) = cands.iter().map(|p| (*p, score(p)))
        .max_by(|a, b| a.1.total_cmp(&b.1)).unwrap();
    if s > best_score { best = c; best_score = s; moves += 1; }
    else { lin *= 0.5; ang *= 0.5; halvings += 1; }
}
``` 
- Check 6 +- 0.05 meters/rad poses around particle.
- MOve to best and repeat. If all worse then halve step size. 
- Continue for 3-5 halvings, ~60 moves total before breaking. 
- Weight by likelyhood at the resulting pose. No drawing form a gaussian. 

## Position Update + Likelyhood (Paper style)
Create priors with unchanged odometry 
### Hill climb
- Check 6 +- 0.05 meters/rad poses around particle.
- MOve to best and repeat. If all worse then halve step size. 
- Continue for 3-5 halvings, need a max move safeguard also make an appripriate max moves per halving so we dont walk too far. 

- Compute a gaussian from a 3x3 grid (~within one dx square+few hundreds of a rad) of samples around the resulting pose. 
- draw new position form gaussian. 







