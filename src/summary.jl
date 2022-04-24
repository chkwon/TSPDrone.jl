function print_summary(result::TSPDroneResult)
    report = generate_summary(result)
    println(report)
end

function generate_summary(result::TSPDroneResult)
    return generate_summary(
        result.truck_route, 
        result.drone_route, 
        result.Ct, 
        result.Cd, 
        flying_range=result.flying_range
    )
end

function fmt(n)
    # return round(n, digits=5)
    return n
end

function generate_summary(
    truck_route, 
    drone_route,
    Ct, 
    Cd;
    flying_range::Float64 = MAX_DRONE_RANGE
)
    combined_nodes = intersect(truck_route, drone_route)
    operations = [(combined_nodes[i], combined_nodes[i+1]) for i in 1:length(combined_nodes)-1]

    summary = String[]
    op_costs = Float64[]

    io = IOBuffer();

    for i in 1:length(operations)
        op = operations[i]
        truck_length, truck_sub_route = operation_length(op[1], op[2], truck_route, Ct)
        drone_length, drone_sub_route = operation_length(op[1], op[2], drone_route, Cd)

        op_length = max(truck_length, drone_length)
        push!(op_costs, op_length)

        flying_range_pass = drone_length < flying_range ? "Pass" : "Fail!!!!"

        println(io, "Operation #$(i):")
        println(io, "  - Truck        = $(fmt(truck_length)) : $(truck_sub_route)")
        println(io, "  - Drone        = $(fmt(drone_length)) : $(drone_sub_route)")
        println(io, "  - Length       = $(fmt(op_length))")
        if drone_length > flying_range
            write(io, "  - flying_range : ")
            printstyled(io, "FAIL\n", color=:red)
        end

    end

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