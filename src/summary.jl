function print_summary(result::TSPDroneResult)
    report = generate_summary(result)
    println(report)
end

fmt(n) = n #round(n, digits=5)

function generate_summary(result::TSPDroneResult)
    combined_nodes = intersect(result.truck_route, result.drone_route)
    operations = [(combined_nodes[i], combined_nodes[i+1]) for i in 1:length(combined_nodes)-1]

    op_costs = Float64[]
    io = IOBuffer()

    for i in 1:length(operations)
        op = operations[i]
        truck_length, truck_sub_route = 
            operation_length(op[1], op[2], result.truck_route, result.Ct)
        drone_length, drone_sub_route = 
            operation_length(op[1], op[2], result.drone_route, result.Cd)

        op_length = max(truck_length, drone_length)
        push!(op_costs, op_length)

        println(io, "Operation #$(i):")
        println(io, "  - Truck        = $(fmt(truck_length)) : $(truck_sub_route)")
        println(io, "  - Drone        = $(fmt(drone_length)) : $(drone_sub_route)")
        println(io, "  - Length       = $(fmt(op_length))")
        if drone_length > result.flying_range
            write(io, "  - flying_range : ")
            printstyled(io, "FAIL\n", color=:red)
        end

    end

    @assert sum(op_costs) == result.total_cost
    
    write(io, "Total Cost = $(fmt(sum(op_costs)))")

    report = String(take!(io))

    return report
end

function operation_length(origin, destination, route, cost_mtx)
    o_idx = findfirst(x -> x == origin, route)
    d_idx = findfirst(x -> x == destination, route)
    sub_route = route[o_idx:d_idx]

    length = 0.0 
    for i in o_idx:d_idx-1
        length += cost_mtx[route[i], route[i+1]]
    end
    
    return length, sub_route   
end