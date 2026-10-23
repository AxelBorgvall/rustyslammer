


# Data forms
## State
Every state has a full precision pose and f_score+cost. For comparison with other states+heuristic grabbing its quantized into a 3d x,y,theta net. 
## Dijkstra heuristic
Numerical Hashmap of chunks the size of x,y quantization. Angle invariant. Permits travelling though unexplored areas but with some additional cost (maybe?). Need to compute at the map resolution but convert down to the internal x,y quantization when done. 

For any states that end up ouside the Dijkstra we use straight euclidian distance + big constant factor. Make dijkstra fairly big so it covers most everythign. 
## Reeds-Shepp Heuristic
Numerical Hashmap of chunks the size of x,y quantization. Doesnt need to be recomputed step to step as it is obstacle invariant. A 100x100 meter square of heuristic around the goal will be about 4MB at dx_huer=0.1 so I reckon its pretty safe to just go huge. If anything ends up outside maybe use a straightline or recompute?
