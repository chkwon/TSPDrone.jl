function find_tsp_tour(Ct::Matrix{Float64})
    scale_factor = 1000
    dist_mtx = round.(Int, Ct .* scale_factor)
    
    tsp_tour, tsp_tour_len = Concorde.solve_tsp(dist_mtx)

    @assert tsp_tour[1] == 1

    return tsp_tour
end


function find_tsp_tour(x::Vector{Float64}, y::Vector{Float64})
    scale_factor = 1000
    
    tsp_tour, tsp_tour_len = Concorde.solve_tsp(x * scale_factor, y * scale_factor; dist="EUC_2D")

    @assert tsp_tour[1] == 1

    return tsp_tour
end

function exact_partitioning(initial_tour, Ct, Cd; flying_range=MAX_DRONE_RANGE)
    n, _ = size(Ct)

    r = initial_tour
    T = fill(Inf, n, n)
    M = fill(-99, n, n)
    for i in 1:n-1
        for j in i+1:n
            if j == i + 1 
                T[i, j] = Ct[r[i], r[j]]
                M[r[i], r[j]] = -1
            else
                for k in i+1:j-1
                    Tk1 = Cd[r[i], r[k]] + Cd[r[k], r[j]]
                    if Tk1 <= flying_range
                        Tk2 = sum([Ct[r[l], r[l+1]] for l in i:k-2]) + 
                                Ct[r[k-1], r[k+1]] + 
                                sum([Ct[r[l], r[l+1]] for l in k+1:j-1])  
                        Tk = max(Tk1, Tk2)
                        if Tk < T[i, j]
                            T[i, j] = Tk
                            M[r[i], r[j]] = r[k]
                        end
                    end
                end       
                  
            end
        end
    end

    V = zeros(n)
    P = fill(-1, n)

    V[1] = 0 
    for i in 2:n 
        VV = [V[k] + T[k, i] for k in 1:i-1]
        V[i] = minimum(VV)
        P[i] = r[argmin(VV)]
    end        

    # Retrieving solutions.
    combined_nodes = Int[]
    current_idx = n
    current = r[current_idx]
    while current != -1 
        pushfirst!(combined_nodes, current)
        current_idx = findfirst(x -> x == current, r)
        current = P[current_idx]
    end
    drone_only_nodes = Int[]
    drone_route = Int[]
    push!(drone_route, combined_nodes[1])
    @assert combined_nodes[1] == r[1]
    for i in 1:length(combined_nodes)-1
        j1 = combined_nodes[i]
        j2 = combined_nodes[i+1]
        if M[j1, j2] != -1 
            push!(drone_only_nodes, M[j1, j2])
            push!(drone_route, M[j1, j2])
        end
        push!(drone_route, j2)
    end
    truck_route = setdiff(initial_tour, drone_only_nodes)

    obj_val = objective_value(truck_route, drone_route, Ct, Cd)
    final_time = V[end]

    @assert isapprox(obj_val, final_time)
    return final_time, truck_route, drone_route
end


function two_point_move(tour, i, j)
    if i >= j 
        return tour, false
    end

    tmp = copy(tour)
    tmp[j] = tour[i]
    tmp[i] = tour[j]
    return tmp, true
end

function one_point_move(tour, i, j)
    tmp = copy(tour)
    deleteat!(tmp, i)
    insert!(tmp, j, tour[i])
    return tmp, true
end

function two_opt_move(tour, i, j)
    if i >= j 
        return tour, false
    end

    tmp = copy(tour)
    tmp[i:j] = tour[j:-1:i]
    return tmp, true
end



# Main function to call
function tsp_ep_all(
    x_coordinates, 
    y_coordinates, 
    truck_cost_factor, 
    drone_cost_factor; 
    local_search_methods=[two_point_move, one_point_move, two_opt_move], 
    flying_range=MAX_DRONE_RANGE, 
    time_limit=MAX_TIME_LIMIT
)    
    """
    Runs `TSP-ep-all` heuristic algorithm of Agatz et al.

    `x_coordinates[1]`, `y_coordinates[1]`: the coordinates of the depot, then followed by all customer location coordinates
    `truck_cost_factor`: as defined in Agatz et al. instances
    `drone_cost_factor`: as defined in Agatz et al. instances
    """

    Ct, Cd = distance_matrices(x_coordinates, y_coordinates, truck_cost_factor, drone_cost_factor)
    n_nodes = length(x_coordinates)
    n1, n2 = size(Ct)
    @assert n_nodes + 1 == n1
    @assert size(Ct) == size(Cd)

    tsp_tour = find_tsp_tour(x_coordinates, y_coordinates)
    push!(tsp_tour, n_nodes+1) # adding a dummy node for the returning depot 

    return tsp_ep_all(Ct, Cd, tsp_tour, flying_range=flying_range, local_search_methods=local_search_methods, time_limit=time_limit)
end


function tsp_ep_all(
    Ct, 
    Cd, 
    init_tour; 
    local_search_methods=[two_point_move, one_point_move, two_opt_move], 
    flying_range=MAX_DRONE_RANGE, 
    time_limit=MAX_TIME_LIMIT
)   
    n, _ = size(Ct)

    improved = true
    best_obj, t_route, d_route = exact_partitioning(init_tour, Ct, Cd, flying_range=flying_range) 

    best_tour = copy(init_tour)
    best_t_route = copy(t_route)
    best_d_route = copy(d_route)

    if isempty(local_search_methods)
        return best_obj, best_t_route, best_d_route
    end

    time0 = time()

    is_time_over = false

    while improved && !is_time_over
        improved = false
        cur_best_obj = best_obj
        cur_best_tour = copy(best_tour)
        cur_best_t_route = copy(best_t_route)
        cur_best_d_route = copy(best_d_route)

        for i in 2:n-1
            if is_time_over
                break
            end

            for local_search in local_search_methods
                for j in 2:n-1
                    if is_time_over
                        break
                    end
                                        
                    new_tour, is_valid = local_search(best_tour, i, j)
                    is_time_over = time() - time0 > time_limit 

                    if is_valid
                        ep_time, t_route, d_route = exact_partitioning(new_tour, Ct, Cd, flying_range=flying_range)
                        if ep_time < cur_best_obj
                            cur_best_tour = copy(new_tour)
                            cur_best_t_route = copy(t_route)
                            cur_best_d_route = copy(d_route)
                            cur_best_obj = ep_time
                            improved = true
                        end
                    end
                end
            end
        end

        if improved 
            best_obj = cur_best_obj 
            best_tour = cur_best_tour
            best_t_route = cur_best_t_route
            best_d_route = cur_best_d_route
        end
    end

    return best_obj, best_t_route, best_d_route
end
