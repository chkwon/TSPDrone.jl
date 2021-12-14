using TSPDrone
using Test, Statistics
# include("../src/main.jl")

include("test_instances.jl")

@testset "TSPDrone.jl" begin
    checkTestInstances()
end
