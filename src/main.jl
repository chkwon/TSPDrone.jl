
using Concorde
using Statistics
using Match

const MAX_TIME_LIMIT = Inf
const MAX_DRONE_RANGE = Inf


include("tspd_utils.jl")
include("tsp_ep_all.jl")
include("DPS.jl")

function solve_tspd(
    x, y, 
    truck_cost_factor, drone_cost_factor; 
    n_groups=1, 
    method="TSP-ep-all", 
    flying_range=MAX_DRONE_RANGE, 
    time_limit=MAX_TIME_LIMIT
)

    local_search_methods = [two_point_move, one_point_move, two_opt_move]
    local_search_methods = @match method begin 
        "TSP-ep"        => []
        "TSP-ep-1p"     => [one_point_move]
        "TSP-ep-2p"     => [two_point_move]
        "TSP-ep-2opt"   => [two_opt_move]
        "TSP-ep-all"    => [two_point_move, one_point_move, two_opt_move]
    end

    return divide_partition_search(x, y, truck_cost_factor, drone_cost_factor; local_search_methods=local_search_methods, n_groups=n_groups, flying_range=flying_range, time_limit=time_limit)
end
