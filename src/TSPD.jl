module TSPD

using Concorde
using Statistics
using Match

const MAX_TIME_LIMIT = Inf


include("tspd_utils.jl")
include("tsp_ep_all.jl")
include("DCH.jl")

function solve_tspd(x, y, speed_truck, speed_drone, flying_range; n_groups=1, method="TSP-ep-all", time_limit=MAX_TIME_LIMIT)
    local_search_methods = [two_point_move, one_point_move, two_opt_move]
    local_search_methods = @match method begin 
        "TSP-ep"        => []
        "TSP-ep-1p"     => [one_point_move]
        "TSP-ep-2p"     => [two_point_move]
        "TSP-ep-2opt"   => [two_opt_move]
        "TSP-ep-all"    => [two_point_move, one_point_move, two_opt_move]
    end

    return divide_conquer(x, y, speed_truck, speed_drone, flying_range; local_search_methods=local_search_methods, method=method, n_groups=n_groups, time_limit=time_limit)
end

export solve_tspd

end