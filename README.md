# TSPDrone.jl


[![Build Status](https://github.com/chkwon/TSPDrone.jl/workflows/CI/badge.svg?branch=master)](https://github.com/chkwon/TSPDrone.jl/actions?query=workflow%3ACI)
[![codecov](https://codecov.io/gh/chkwon/TSPDrone.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/chkwon/TSPDrone.jl)

This package solves the Traveling Salesman Problem with Drone (TSP-D) with 1 truck and 1 drone. This implements the Divide-Partition-and-Search (DPS) algorithm and the Deep Reinforcement Learning (DRL) method as proposed in the following paper:

* [Bogyrbaeyeva A., T. Yoon, H. Ko, S. Lim, H. Yun, and C. Kwon, A Deep Reinforcement Learning Approach for Solving the Traveling Salesman Problem with Drone, working paper](https://arxiv.org/abs/2112.12545). 

The DPS algorithm is based on the TSP-ep-all algorithm of [Agatz et al. (2018)](https://doi.org/10.1287/trsc.2017.0791) and the divide-and-conquer heuristic of [Poikonen et al. (2019)](https://doi.org/10.1287/ijoc.2018.0826).

If you use either the DPS algorithm or the DRL method, pleaes cite:
```
@misc{bogyrbayeva2021deep,
      title={A Deep Reinforcement Learning Approach for Solving the Traveling Salesman Problem with Drone}, 
      author={Aigerim Bogyrbayeva and Taehyun Yoon and Hanbum Ko and Sungbin Lim and Hyokun Yun and Changhyun Kwon},
      year={2021},
      eprint={2112.12545},
      archivePrefix={arXiv},
      primaryClass={math.OC}
}
```

# Install

```julia
] add https://github.com/chkwon/TSPDrone.jl
```

If you want just the DPS algorithm, the above will be sufficient. Skip the rest.

If you also want to use the DRL method, you need to set up your Python and PyTorch installations. 
Suppose your Python installation is located at `/usr/local/bin/python3`.
First, make sure that this particular Python has `torch`, `numpy`, and `scipy` packages; if not, please install them. 
For example:
```
python3 -m pip install torch numpy scipy
```

In Julia:
```
julia> ENV["PYTHON"] = "/usr/local/bin/python3"
```
Then
```
julia> import Pkg; Pkg.build("PyCall")
```
Test if everything works fine:
```
julia> using TSPDrone
julia> TSPDrone.test_RL()
```
which should not generate errors.
If it does not work properly, check if your Julia is connected with a proper Python installation. 
For example:
```
julia> using PyCall
julia> PyCall.python
"/usr/local/bin/python3"
julia> PyCall.pyversion
v"3.9.7"
```
If it does not use the Python installation you like, try the above process again.


# Using the Divide-Partition-and-Search (DPS) Algorithm

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
returns
```
(3.268788924769125, [1, 5, 2, 6, 4, 7, 3, 11], [1, 10, 6, 8, 4, 9, 3, 11])
```
where again node `11` represets the depot as the final destination.

## Options for DPS 
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



# Using the Deep Reinforcement Learning (DRL) method

This package comes with pre-trained weights for `n ∈ [11, 15, 20, 50, 100]`; these sizes include the depot.
For training, the followings were used:
- The x,y coordinate of the depot is uniformly distributed in `[0, 1]`.
- The x,y coordinates of the customers are uniformly distributed in `[0, 100]`.
- `truck_cost_factor = 1.0` and `drone_cost_factor = 0.5`.
- The drone fyling range is ∞.

For example, 
```julia
# the problem size, n ∈ [11, 15, 20, 50, 100]
n = 11 

# depot coordinates from Uniform[0, 1]
depot_x = rand()
depot_y = rand()

# customer coorindates from Uniform[0, 100]
customers_x = rand(n - 1) .* 100
customers_y = rand(n - 1) .* 100

# the first elements are for the depot
x_coordinates = [depot_x; customers_x]
y_coordinates = [depot_y; customers_y]

@assert n == length(x_coordinates) == length(y_coordinates)

using TSPDrone
obj, truck_route, drone_route = solve_tspd_RL(x_coordinates, y_coordinates; n_samples = 100)
```

In a sample run, the outcome was:
```
(239.78793778400822, [1, 3, 8, 2, 11, 6, 7, 12], [1, 10, 3, 5, 8, 9, 6, 4, 12])
```
In the truck and drone routes, both node 1 and node 12 refers to the depot. 

## Options for DRL
```julia
obj, truck_route, drone_route = solve_tspd_RL(x_coordinates, y_coordinates; n_samples = 100, device = "cpu")
```

If `n_samples = 1` or not provided, it will use the greedy decoding.
If `n_samples > 1`, then it will generate multiple samples and return the best result.

You can also pass `device`. The default is `device = "cpu"`, but if you have a GPU, then you can also pass the device name, so that `torch` can run on the GPU. 
