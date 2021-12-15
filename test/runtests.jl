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
            obj1, tr1, dr1 = solve_tspd(x, y, 1.0, 0.5)
            obj2, tr2, dr2 = solve_tspd(dist_mtx, dist_mtx .* 0.5)

            @show obj1, tr1, dr1
            @show obj2, tr2, dr2

            @test obj1 == obj2 
            # @test tr1 == tr2 
            # @test dr1 == dr2 
        end
    end

    @testset verbose = true "Test Instances" begin
        checkTestInstances()
    end

end

