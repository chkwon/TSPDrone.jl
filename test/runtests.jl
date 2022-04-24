using TSPDrone
using Test, Statistics
# include("../src/main.jl")

include("test_instances.jl")

@testset verbose = true "TSPDrone.jl" begin

    @testset verbose = true "Cost Matrix" begin
        for t = 1:10
            n = 10 
            x = rand(n)
            y = rand(n)
            dist_mtx = zeros(n, n)
            for i in 1:n, j in 1:n
                dist_mtx[i, j] = sqrt( (x[i] - x[j])^2 +(y[i] - y[j])^2 )
            end
            result1 = solve_tspd(x, y, 1.0, 0.5)
            result2 = solve_tspd(dist_mtx, dist_mtx .* 0.5)

            @show result1.total_cost, result1.truck_route, result1.drone_route
            @show result2.total_cost, result2.truck_route, result2.drone_route

            @test result1.total_cost ≈ result2.total_cost 
            # @test tr1 == tr2 
            # @test dr1 == dr2 
        end
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

