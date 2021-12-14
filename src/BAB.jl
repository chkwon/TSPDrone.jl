# Branch and Bound for TSPD by Poikonen et al. 2019

# include("tsp_ep_all.jl")

mutable struct BranchNode
    seq :: Vector{Int}
    LB  :: Float64
    UB  :: Float64
end


function create_children(node::BranchNode, Ct::Matrix{Float64}, Cd::Matrix{Float64}, n_nodes)
    if length(node.seq) == n_nodes + 1
        return BranchNode[] 
    end

    sequence = node.seq 
    farthest_node = -1
    farthest_distance = 0.0
    for i in 2:n_nodes 
        if ! in(i, node.seq)
            dist = 0.0
            for j in node.seq
                dist = max(Ct[i, j], dist)
            end

            if dist > farthest_distance
                farthest_node = i 
                farthest_distance = dist 
            end
        end
    end

    # Insert and generate children
    n_children = length(node.seq) - 1
    child_nodes = Vector{BranchNode}(undef, n_children)
    for i in 1:n_children
        new_seq = copy(node.seq)
        insert!(new_seq, i+1, farthest_node)
        child_nodes[i] = BranchNode(new_seq, -Inf, Inf)
        update_bounds!(child_nodes[i], Ct, Cd, n_nodes)
    end

    return child_nodes
end

function update_bounds!(node::BranchNode, Ct, Cd, n_nodes)

    Ct_ = Ct[node.seq, node.seq]
    Cd_ = Cd[node.seq, node.seq]
    len_seq = length(node.seq)
    ALB = exact_partitioning(1:len_seq, len_seq-1, Ct_, Cd_, method="Poikonen")
    node.LB = ALB 

    if length(node.seq) <= n_nodes 
        node.UB = Inf
    elseif length(node.seq) == n_nodes + 1
        node.UB = ALB
    else
        error("Node.seq is too long.")
    end
end

function branch_and_bound(Ct, Cd, n_nodes; TER = 1.0)
    n1, n2 = size(Ct)
    @assert n_nodes == n1 - 1
    @assert size(Ct) == size(Cd)


    root_node = BranchNode([1, 2, 3, n_nodes+1], -Inf, Inf) 
    update_bounds!(root_node, Ct, Cd, n_nodes)  
    best_LB = -Inf 
    best_UB = Inf 

    function branching(node::BranchNode)
        best_LB = max(best_LB, node.LB)
        best_UB = min(best_UB, node.UB)

        ratio = node.LB / best_UB :: Float64
        if ratio >= TER 
            return -1
        end

        children = create_children(node, Ct, Cd, n_nodes)
        sort!(children, by = x -> x.LB)
        for child in children 
            branching(child)
        end
        return -1
    end



    branching(root_node)

    obj_val = best_UB ::Float64
    return obj_val
end

function branch_and_bound(x, y, truck_cost_factor, drone_cost_factor; TER = 1.0)

    Ct, Cd = distance_matrices(x, y, truck_cost_factor, drone_cost_factor)
    n_nodes = length(x)
    n1, n2 = size(Ct)
    @assert n_nodes + 1 == n1
    @assert size(Ct) == size(Cd)

    return branch_and_bound(Ct, Cd, n_nodes; TER=TER)
end