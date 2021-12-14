push!(LOAD_PATH, joinpath(@__DIR__, "src"))

MAX_TIME_LIMIT = 3600.0

using TSPD 

MAX_TIME_LIMIT = 3600.0

function test_new_data(dir_name, n; method="TSP-ep-all", n_groups=1, time_limit=MAX_TIME_LIMIT)

    truck_cost_factor = 1
    drone_cost_factor = 0.5

    filename = joinpath(@__DIR__, "InstancesSolutions/$(dir_name)/Concorde-TSP-len-100-n_nodes-$(n).txt")
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
        obj, _, _ = solve_tspd(x, y, truck_cost_factor, drone_cost_factor, method=method, n_groups=n_groups, time_limit=time_limit)
        push!(objs, obj)
    end
    t1 = time()
    total_time = t1 - t0
    avg_time = total_time / batch_size
    avg_obj = sum(objs) / length(objs)
    @show dir_name, n, avg_time, method, n_groups, time_limit, avg_obj

    sol_filename = joinpath(@__DIR__, "InstancesSolutions/$(dir_name)/Agatz-heuristic-len-100-n_nodes-$(n)-julia-$(method)-n_groups-$(n_groups)-time_limit-$(time_limit).txt")
    time_filename = joinpath(@__DIR__, "InstancesSolutions/$(dir_name)/Agatz-heuristic-runtime-len-100-n_nodes-$(n)-julia-$(method)-n_groups-$(n_groups)-time_limit-$(time_limit).txt")

    open(sol_filename, "w") do io 
        for i in 1:batch_size
            println(io, objs[i])
        end
    end
    open(time_filename, "w") do io 
        println(io, total_time)
        println(io, avg_time)
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



test_new_data("n11", 11)
# test_new_data("n11", 11)


# test_new_data("n15", 15)
# test_new_data("n20", 20)
# test_new_data("n20", 20, method="TSP-ep-all", n_groups=2)
# test_new_data("n50", 50, method="TSP-ep-all", n_groups=5)
# test_new_data("n100", 100, method="TSP-ep-all", n_groups=10)
# test_new_data("n100float", 100, method="TSP-ep-all", n_groups=10)


# test_new_data("n50float", 50, method="TSP-ep-all", n_groups=2, time_limit=3600.0)

# test_new_data("n50float", 50, method="TSP-ep-all", n_groups=1, time_limit=3600.0)

# test_new_data("n100float", 100, method="TSP-ep-all", n_groups=4, time_limit=3600.0)

# test_new_data("n100float", 100, method="TSP-ep-all", n_groups=1, time_limit=7200.0)

# test_new_data("n50float", 50, method="TSP-ep-all", n_groups=2, time_limit=3600.0)

# test_new_data("n50float", 50, method="TSP-ep-all", n_groups=1, time_limit=3600.0)


test_new_data("n100float", 100, method="TSP-ep-all", n_groups=5, time_limit=3600.0)

test_new_data("n100float", 100, method="TSP-ep-all", n_groups=2, time_limit=3600.0)
