# TSPDrone.jl


[![Build Status](https://github.com/chkwon/TSPDrone.jl/workflows/CI/badge.svg?branch=master)](https://github.com/chkwon/TSPDrone.jl/actions?query=workflow%3ACI)
[![codecov](https://codecov.io/gh/chkwon/TSPDrone.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/chkwon/TSPDrone.jl)

This package solves the Traveling Salesman Problem with Drone (TSP-D) with 1 truck and 1 drone. This implements the Divide-Partition-and-Search (DPS) algorithm as proposed in the following paper:

* Bogyrbaeyeva A., T. Yoon, H. Ko, S. Lim, H. Yun, and C. Kwon, A Deep Reinforcement Learning Approach for Solving the Traveling Salesman Problem with Drone, working paper. 

The DPS algorithm is based on the TSP-ep-all algorithm of [Agatz et al. (2018)](https://doi.org/10.1287/trsc.2017.0791) and the divide-and-conquer heuristic of [Poikonen et al. (2019)](https://doi.org/10.1287/ijoc.2018.0826).


# Usage

You can provide `x` and `y` coordinates of customers. 
The depot should be the first element in `x` and `y`.

Two parameters `truck_cost_factor` and `drone_cost_factor` will be multiplied to the Euclidean distance calculated from the coordinates. 
```julia 
n = 10 
x = rand(n); y = rand(n);
truck_cost_factor = 1.0 
drone_cost_factor = 0.5
objective_value, truck_route, drone_route = solve_tspd(x, y, truck_cost_factor, drone_cost_factor)
```
returns
```
(1.8835545178380921, [1, 5, 9, 3, 8, 7, 11], [1, 4, 5, 6, 8, 10, 7, 2, 11])
```
where node `11` represents the depot as the final destination. 

You can also provide the cost matrices of truck and drone directly.
Again, the depot is labeled as `1`.
```julia
n = 10 
dist_mtx = rand(n, n)
dist_mtx = dist_mtx + dist_mtx' # symmetric distance only
truck_cost_mtx = dist_mtx .* 1.0
drone_cost_mtx = truck_cost_mtx .* 0.5 
sol, tr, dr = solve_tspd(truck_cost_mtx, drone_cost_mtx)
@assert size(truck_cost_mtx) == size(drone_cost_mtx) == (n, n)
```

# Options for DPS 
Optional keyword arguments for `solve_tspd`:
```julia
n_groups::Int = 1, 
method::String = "TSP-ep-all", 
flying_range::Float64 = MAX_DRONE_RANGE, 
time_limit::Float64 = MAX_TIME_LIMIT
```
* `n_groups`: The number of subgroups for divide and conquer. For example, if `n=100` and `n_groups=4`, then each group will have `25` nodes. Then, `method` is applied for each group. 
* `method`: can be one of the following (See [Agatz et al. 2018](https://doi.org/10.1287/trsc.2017.0791)):
    - `"TSP-ep"`: The exact partitioning of the initial TSP tour is performend. Then, no search is performed.
    - `"TSP-ep-1p"`: The exact partitioning of the initial TSP tour is performend. Then, all possible one-point moves are performed on the initial TSP tour, which is followed by exact partitioning of new TSP tours.
    - `"TSP-ep-2p"`: All possible two-point moves.
    - `"TSP-ep-2opt"`: All possible 2-opt exchange moves.
    - `"TSP-ep-all"`: All possible 1p, 2p, and 2opt moves.
* `flying_range`: The limited flying range of the drone. The default value is `Inf`. The flying range is compared with the values in the drone cost matrix; that is, `drone_cost_mtx` or the Euclidean distance multiplied by `drone_cost_factor`. 
* `time_limit`: The total time limit to solve the problem in seconds. For each group, the time limit is split equally. For example, if `time_limit=3600.0` and `n_groups=5`, then each group has time limit of 3600/5 = 720 seconds. 

