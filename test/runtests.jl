using TSPDrone
using Test, Statistics
# include("../src/main.jl")

include("test_instances.jl")

@testset verbose = true "TSPDrone.jl" begin

    @testset verbose = true "Cost Matrix" begin
        n = 10 
        x = rand(n)
        y = rand(n)
        Ct, Cd = TSPDrone.distance_matrices(x, y, 1.0, 0.5)
        sol, tr, dr = solve_tspd(Ct, Cd)
        @show sol 
        @show tr 
        @show dr
        @test typeof(sol) == Float64 
        @test typeof(tr) == Vector{Int}
        @test typeof(dr) == Vector{Int}
    end

    @testset verbose = true "Test Instances" begin
        checkTestInstances()
    end

end

