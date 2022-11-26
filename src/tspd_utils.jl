
function cost_matrices_with_dummy(truck_cost_mtx::Matrix{Float64}, drone_cost_mtx::Matrix{Float64})
    Ct = [
        truck_cost_mtx          truck_cost_mtx[:, 1];
        truck_cost_mtx[1, :]'    0.0
    ]

    Cd = [
        drone_cost_mtx          drone_cost_mtx[:, 1];
        drone_cost_mtx[1, :]'    0.0
    ]

    return Ct, Cd
end


function _cost_matrices_with_dummy(x::Vector{Float64}, y::Vector{Float64}, speed_of_truck::Float64, speed_of_drone::Float64)
    n_nodes = length(x)
    @assert length(x) == length(y)

    dist = zeros(Float64, n_nodes, n_nodes)
    for i in 1:n_nodes
        for j in 1:n_nodes
            dist[i, j] = (x[i]-x[j])^2 + (y[i]-y[j])^2 |> sqrt 
        end
    end

    Ct = speed_of_truck .* dist 
    Cd = speed_of_drone .* dist 

    return Ct, Cd
end

function cost_matrices_with_dummy(x::Vector{Float64}, y::Vector{Float64}, speed_of_truck::Float64, speed_of_drone::Float64)
    xx = copy(x)
    yy = copy(y)
    push!(xx, x[1])
    push!(yy, y[1])
    return _cost_matrices_with_dummy(xx, yy, speed_of_truck, speed_of_drone)
end



function travel_cost(path::Vector{Int}, C::Matrix{T}) where T
    # @show path
    sum = zero(T)
    for i in 1:length(path)-1
        sum += C[path[i], path[i+1]]
    end
    return sum
end

function objective_value(truck_route::Vector{Int}, drone_route::Vector{Int}, Ct::Matrix{Float64}, Cd::Matrix{Float64})
    combined_nodes = intersect(truck_route, drone_route)
    obj_val = 0.0
    for i in 1:length(combined_nodes)-1
        j1 = combined_nodes[i]
        j2 = combined_nodes[i+1]
        
        t_idx1 = findfirst(x -> x == j1, truck_route)
        t_idx2 = findfirst(x -> x == j2, truck_route)
        t_cost = travel_cost(truck_route[t_idx1:t_idx2], Ct)

        d_idx1 = findfirst(x -> x == j1, drone_route)
        d_idx2 = findfirst(x -> x == j2, drone_route)
        d_cost = travel_cost(drone_route[d_idx1:d_idx2], Cd)

        obj_val += max(t_cost, d_cost)
    end
    return obj_val
end
