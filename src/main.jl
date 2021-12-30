
using Concorde
using Statistics
using Match
using PyCall

const MAX_TIME_LIMIT = Inf
const MAX_DRONE_RANGE = Inf


include("tspd_utils.jl")
include("tsp_ep_all.jl")
include("DPS.jl")
include("PythonRL/wrapper.jl")

function local_search_functions(method::String)
    local_search_methods = Function[two_point_move, one_point_move, two_opt_move]
    local_search_methods = @match method begin 
        "TSP-ep"        => Function[]
        "TSP-ep-1p"     => Function[one_point_move]
        "TSP-ep-2p"     => Function[two_point_move]
        "TSP-ep-2opt"   => Function[two_opt_move]
        "TSP-ep-all"    => Function[two_point_move, one_point_move, two_opt_move]
    end
    return local_search_methods
end

function solve_tspd(
    x::Vector{Float64}, 
    y::Vector{Float64}, 
    truck_cost_factor::Float64, 
    drone_cost_factor::Float64; 
    n_groups::Int = 1, 
    method::String = "TSP-ep-all", 
    flying_range::Float64 = MAX_DRONE_RANGE, 
    time_limit::Float64 = MAX_TIME_LIMIT
)


    local_search_methods = local_search_functions(method)
    Ct, Cd = cost_matrices_with_dummy(x, y, truck_cost_factor, drone_cost_factor)

    return divide_partition_search(Ct, Cd; local_search_methods=local_search_methods, n_groups=n_groups, flying_range=flying_range, time_limit=time_limit)
end


function solve_tspd(
    truck_cost_mtx::Matrix{Float64}, 
    drone_cost_mtx::Matrix{Float64};
    n_groups::Int = 1, 
    method::String = "TSP-ep-all", 
    flying_range::Float64 = MAX_DRONE_RANGE, 
    time_limit::Float64 = MAX_TIME_LIMIT
)

    Ct = [
        truck_cost_mtx          truck_cost_mtx[:, 1];
        truck_cost_mtx[1, :]'    0.0
    ]

    Cd = [
        drone_cost_mtx          drone_cost_mtx[:, 1];
        drone_cost_mtx[1, :]'    0.0
    ]

    local_search_methods = local_search_functions(method)

    return divide_partition_search(Ct, Cd; local_search_methods=local_search_methods, n_groups=n_groups, flying_range=flying_range, time_limit=time_limit)
end
