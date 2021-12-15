# TSPDrone.jl


[![Build Status](https://github.com/chkwon/TSPDrone.jl/workflows/CI/badge.svg?branch=master)](https://github.com/chkwon/TSPDrone.jl/actions?query=workflow%3ACI)
[![codecov](https://codecov.io/gh/chkwon/TSPDrone.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/chkwon/TSPDrone.jl)

This package solves the Traveling Salesman Problem with Drone (TSP-D) with 1 truck and 1 drone. This implements the Divide-Partition-and-Search (DPS) algorithm as proposed in the following paper:

* Bogyrbaeyeva A., T. Yoon, H. Ko, S. Lim, H. Yun, and C. Kwon, A Deep Reinforcement Learning Approach for Solving the Traveling Salesman Problem with Drone, working paper. 

The DPS algorithm is based on the TSP-ep-all algorithm of [Agatz et al. (2018)](https://doi.org/10.1287/trsc.2017.0791) and the divide-and-conquer heuristic of [Poikonen et al. (2019)](https://doi.org/10.1287/ijoc.2018.0826).


# Usage

You can provide `x` and `y` coordinates of customers. 
The depot should be the first element and 


```julia
n = 10 
x = rand(n); y = rand(n);
Ct, Cd = TSPDrone.distance_matrices(x, y, 1.0, 0.5)
sol, tr, dr = solve_tspd(Ct, Cd)
@assert size(Ct) == size(Cd) == (n+1, n+1)
@show sol 
@show tr 
@show dr
```