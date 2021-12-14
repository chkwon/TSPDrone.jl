This directory includes new test instances for TSP-D. 

The folder name, for example `n11`, indicates the number of customer nodes plus the depot. Inside, there are files for 100 test instances:

#### `TSPD_n11_instances.txt`

Each row represents a test instance, where we follow the format of 
 
```
x1 y1 x2 y2 ... xn yn
```
    
where `x1 y1` are for the depot.

#### `TSPD_n11_TSP_ep_all_solutions.txt`

Each row provides the objective value found by TSP_ep_all. 

Since the Divide-Partition-and-Search (DPS) results can be quickly obtained by running the package, the solutions are not provided.
