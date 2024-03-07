using TSPDrone
using Test, Statistics

include("../src/tspd_utils.jl")
include("../src/tsp_ep_all.jl")

const MAX_FLIGHT_RANGE = Inf

@testset verbose=true "EP Complexity Comparison" begin
    total_time_1 = 0.0
    total_time_2 = 0.0
    iterations = 10
    for t = 1:iterations
        n = 200
        x = rand(n)
        y = rand(n)
        
        dist_mtx = sqrt.((x .- x').^2 .+ (y .- y').^2)
        Ct = copy(dist_mtx)
        Cd = copy(dist_mtx) .* 0.5

        tsp_tour = TSPDrone.find_tsp_tour(x, y)
        push!(tsp_tour, n+1)
        # @show tsp_tour

        tsp_tour_1 = copy(tsp_tour)
        time_1 = @elapsed best_obj_1, t_route_1, d_route_1 = exact_partitioning(tsp_tour_1, Ct, Cd, flying_range=MAX_FLIGHT_RANGE, complexity=3)
        total_time_1 += time_1
        @show best_obj_1, t_route_1, d_route_1

        tsp_tour_2 = copy(tsp_tour)
        time_2 = @elapsed best_obj_2, t_route_2, d_route_2 = exact_partitioning(tsp_tour_2, Ct, Cd, flying_range=MAX_FLIGHT_RANGE)
        total_time_2 += time_2
        @show best_obj_2, t_route_2, d_route_2
        
        @test best_obj_1 == best_obj_2
        @test t_route_1 == t_route_2
        @test d_route_1 == d_route_2
    end
    avg_time_1 = total_time_1 / iterations
    avg_time_2 = total_time_2 / iterations
    println("Average time for O(n^3) exact_partitioning: $avg_time_1")
    println("Average time for O(n^4) exact_partitioning: $avg_time_2")
end