using TSPDrone
using Test, Statistics

include("../src/main.jl")
include("test_instances.jl")

@testset verbose = true "TSPDrone.jl" begin

    @testset verbose = true "Cost Matrix" begin
        for t = 1:10
            n = 10 
            x, y, dist_mtx = generate_random_points(n)
            
            result1 = solve_tspd(x, y, 1.0, 0.5)
            result2 = solve_tspd(dist_mtx, dist_mtx .* 0.5)

            @show result1.total_cost, result1.truck_route, result1.drone_route
            @show result2.total_cost, result2.truck_route, result2.drone_route

            @test isapprox(result1.total_cost, result2.total_cost, atol=1e-5)
        end
    end

    @testset verbose = true "EP Complexity Comparison" begin
        total_time_1 = 0.0
        total_time_2 = 0.0
        iterations = 10
        for t = 1:iterations
            n = 100
            x, y, dist_mtx = generate_random_points(n)
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
            
            @test isapprox(best_obj_1, best_obj_2, atol=1e-5)
            # @test t_route_1 == t_route_2
            # @test d_route_1 == d_route_2
        end
        avg_time_1 = total_time_1 / iterations
        avg_time_2 = total_time_2 / iterations
        println("Average time for O(n^3) exact_partitioning: $avg_time_1")
        println("Average time for O(n^4) exact_partitioning: $avg_time_2")
    end

    @testset verbose = true "Test Instances" begin
        checkTestInstances()
    end

    @testset verbose = true "print_summary" begin 
        n = 10 
        x = rand(n); y = rand(n);
        truck_cost_factor = 1.0 
        drone_cost_factor = 0.5
        result = solve_tspd(x, y, truck_cost_factor, drone_cost_factor)
        print_summary(result)
        report = generate_summary(result)
        @test length(split(report, "\n")) > 10
    end

end

