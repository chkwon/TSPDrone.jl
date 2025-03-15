push!(LOAD_PATH, @__DIR__)

using TSPD
using Statistics

function read_data(filename, n)
    dir_path = @__DIR__ 
    inst_dir = "../../ExactSolutions/instances/n$(n)/"
    filepath = joinpath(dir_path, inst_dir, filename)

    f = open(filepath, "r")
    readline(f)
    speed_of_truck = parse(Float64, readline(f))
    readline(f)
    speed_of_drone = parse(Float64, readline(f))
    readline(f)
    n_nodes = parse(Int, readline(f))
    readline(f)
    depot_line = split(readline(f))
    depot_x = parse(Float64, depot_line[1])
    depot_y = parse(Float64, depot_line[2])
    readline(f)
    x = Vector{Float64}(undef, n_nodes)
    y = Vector{Float64}(undef, n_nodes)
    x[1] = depot_x
    y[1] = depot_y
    for i in 2:n_nodes
        cx, cy, _ = split(readline(f))
        x[i] = parse(Float64, cx)
        y[i] = parse(Float64, cy)
    end

    return x, y, speed_of_truck, speed_of_drone
end



exact_opt_obj = Dict{Int, Vector{Float64}}()
# n11
exact_opt_obj[11] = Float64[
    221.18876576478925,
    205.76050725572097,
    192.96313461174037,
    241.25592289521398,
    248.1379946498235,
    217.68894293889753,
    237.34013623078425,
    214.76536428997835,
    256.33972821148967,
    227.90300661076967        
]

# n15
exact_opt_obj[15] = Float64[
    260.19649903254805,  
    279.8748849328301,
    278.2501294150187,
    227.4927030824479,
    233.0058389136502,
    252.9062262214419,
    288.5045658852292,
    277.4202725739683,
    280.23702686030765,
    271.52779790946164         
]

function test_agatz(n)
    println("Testing for n =", n)

    obj_val = Float64[]
    for i in 1:10
        filename = "uniform-$(i)-n$(n).txt"
        tsp_x, tsp_y, truck_cost_factor, drone_cost_factor = read_data(filename, n)
        
        val, _, _ = solve_tspd(tsp_x, tsp_y, truck_cost_factor, drone_cost_factor, method="TSP-ep-all")

        println(val)
        push!(obj_val, val)
    end

    opt_val = exact_opt_obj[n] :: Vector{Float64}
    gap = (obj_val - opt_val) ./ opt_val * 100
    println("average gap = ", mean(gap), " %")
    println("maximum gap = ", maximum(gap), " %")
end

println("-----------------------------")
t0 = time()
test_agatz(11)
t1 = time()
println("average time = ", (t1-t0) / 10, " seconds")
println("-----------------------------")
t0 = time()
test_agatz(15)
t1 = time()
println("average time = ", (t1-t0) / 10, " seconds")
println("-----------------------------")

println("Just testing for push")

# let
#     x = rand(1:100, 10) .* 1.0
#     y = rand(1:100, 10) .* 1.0
#     x[1] = rand()
#     y[1] = rand()
#     @show tspd_heuristic(x, y, 1, 0.5, method="TSP-ep")
#     @show tspd_heuristic(x, y, 1, 0.5, method="TSP-ep-1p")
#     @show tspd_heuristic(x, y, 1, 0.5, method="TSP-ep-2p")
#     @show tspd_heuristic(x, y, 1, 0.5, method="TSP-ep-2opt")
#     @show tspd_heuristic(x, y, 1, 0.5, method="TSP-ep-all", ep="Agatz")
#     @show tspd_heuristic(x, y, 1, 0.5, method="TSP-ep-all", ep="Poikonen")
#     @show branch_and_bound(x, y, 1, 0.5)
# end

# aep_vals = Float64[]
# pep_vals = Float64[]
# bab_vals = Float64[]
# for k in 1:10
#     x = rand(1:100, 11) .* 1.0
#     y = rand(1:100, 11) .* 1.0
#     x[1] = rand()
#     y[1] = rand()
#     aep, _, _ = tspd_heuristic(x, y, 1, 0.5, method="TSP-ep-all", ep="Agatz")
#     pep, _, _ = tspd_heuristic(x, y, 1, 0.5, method="TSP-ep", ep="Poikonen")
#     bab, _, _ = branch_and_bound(x, y, 1, 0.5)

#     push!(aep_vals, aep)
#     push!(pep_vals, pep)
#     push!(bab_vals, bab)
# end
# @show mean(aep_vals), mean(pep_vals), mean(bab_vals)