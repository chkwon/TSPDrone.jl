
function checkTestInstances(dir_name; method="TSP-ep-all", n_groups=1, flying_range=Inf, time_limit=Inf)
    @info "Testing $dir_name, method=$method, n_groups=$n_groups, flying_range=$flying_range, time_limit=$time_limit"

    n = parse(Int, match(r"[0-9]+", dir_name).match)

    truck_cost_factor = 1.0
    drone_cost_factor = 0.5

    filename = joinpath(@__DIR__, "../TestInstances/$(dir_name)/TSPD_n$(n)_instances.txt")
    f = open(filename, "r")
    lines = readlines(f)
    batch_size = length(lines)

    objs = Float64[]

    t0 = time()
    for line in lines 
        data = parse.(Float64, split(line))
        mat = reshape(data, 2, :)
        x = mat[1, :]
        y = mat[2, :]
        result = solve_tspd(x, y, truck_cost_factor, drone_cost_factor, flying_range=flying_range, method=method, n_groups=n_groups, time_limit=time_limit)
        push!(objs, result.total_cost)
    end
    t1 = time()
    total_time = t1 - t0
    avg_time = total_time / batch_size
    avg_obj = mean(objs)
    @show dir_name, n, avg_time, method, n_groups, time_limit, avg_obj

    solution_filename = joinpath(@__DIR__, "../TestInstances/$(dir_name)/TSPD_n$(n)_TSP_ep_all_solutions.txt")
    ff = open(solution_filename, "r")
    sol_lines = readlines(ff)
    sol_objs = parse.(Float64, sol_lines)

    # result_filename = joinpath(@__DIR__, "result-n_nodes-$(n)-$(method)-n_groups-$(n_groups)-flying_range-$(flying_range)-time_limit-$(time_limit).txt")
    # open(result_filename, "w") do io 
    #     for i in 1:batch_size
    #         println(io, objs[i])
    #     end
    # end

    
    gap = 0.01
    if flying_range < Inf 
        gap = 1.0
    elseif method != "TSP-ep-all"
        gap = 1.0
    elseif n_groups > 1 
        gap = 0.10
    end
    @test isapprox(mean(objs), mean(sol_objs); atol = mean(sol_objs) * gap)


    # Each individual result test
    # Every time it runs, it produces slightly different results 
    # It seems that it depends on how Concorde returns the optimal TSP tour. 
    if method == "TSP-ep-all" && n_groups == 1 && flying_range == Inf
        gap = 0.10
        for i in eachindex(objs)
            @test isapprox(objs[i], sol_objs[i]; atol = sol_objs[i] * gap)
        end
    end

    
end



# IF method is not specified, it uses TSP-ep-all
# method="TSP-ep", 
# method="TSP-ep-1p"
# method="TSP-ep-2p"
# method="TSP-ep-2opt"
# method="TSP-ep-all"
# n_groups is the number of groups to be used in the divide-and-conquer framework.

# Example:
# test_new_data("n50", 50, method="TSP-ep-all", n_groups=5, time_limit=3600.0)
# `time_limit` is applied for each instance (in seconds).
# If there are 100 instances, then the total time would be around 100 * 3600.0 seconds.
# For each group, the time limit is split equally.
# If `n_groups == 5`, then each group has time limit of 3600/5 = 720 seconds. 

function checkTestInstances()

    @testset verbose = true "TSP-ep methods for n11" begin
        available_methods = ["TSP-ep", "TSP-ep-1p", "TSP-ep-2p", "TSP-ep-2opt", "TSP-ep-all"]
        for m in available_methods
            @testset "$m" begin
                checkTestInstances("n11"; n_groups=1, method=m, time_limit=10.0)
            end
        end
    end 

    instances = Dict(
        "n20float" => 2,
        "n50float" => 5,
        "n100float" => 10
    )

    for (name, n_groups) in instances 
        @testset verbose = true "DPS for $name" begin
            checkTestInstances(name; n_groups=n_groups, time_limit=1000.0)
        end
    end

    @testset verbose = true "Flying range" begin
        checkTestInstances("n11"; n_groups=1, method="TSP-ep-all", time_limit=10.0, flying_range=50.0)
    end

end