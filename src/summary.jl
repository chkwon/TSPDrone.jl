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
            operation_length(op[1], op[2], result.drone_route, result.Cd; is_drone=true)

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

    @assert isapprox(sum(op_costs), result.total_cost; atol=1e-6)
    
    write(io, "Total Cost = $(fmt(sum(op_costs)))")

    report = String(take!(io))

    return report
end

function operation_length(origin, destination, route, cost_mtx; is_drone=false)
    o_idx = findfirst(x -> x == origin, route)
    d_idx = findfirst(x -> x == destination, route)
    sub_route = route[o_idx:d_idx]

    length = 0.0 
    if is_drone && d_idx == o_idx + 1
        # drone is moving on the truck, so the drone length = 0.0
    else
        for i in o_idx:d_idx-1
            length += cost_mtx[route[i], route[i+1]]
        end
    end

    return length, sub_route   
end