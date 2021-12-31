
function solve_tspd_RL_test(n_nodes, n_groups, n_samples)
    n_runs = 10
    obj_DPS = zeros(n_runs)
    obj_RL = zeros(n_runs)
    for i in 1:n_runs
        x = rand(n_nodes) .* 100
        y = rand(n_nodes) .* 100
        x[1] = rand() # depot
        y[1] = rand() # depot
        
        # x = [0.8172268241831585, 73, 61, 44, 74, 23, 9, 51, 15, 9, 56]
        # y = [0.6284331187597952, 52, 46, 63, 15, 23, 80, 65, 97, 74, 84]
        
        obj_DPS[i], _, _ = solve_tspd(x, y, 1.0, 0.5; n_groups=n_groups)
        obj_RL[i], _, _ = solve_tspd_RL(x, y; n_samples=n_samples)
    end
    pct_gap = (mean(obj_RL) - mean(obj_DPS)) / mean(obj_DPS)
    @show mean(obj_DPS), mean(obj_RL), pct_gap
    @assert pct_gap < 0.05
    return mean(obj_DPS), mean(obj_RL), pct_gap
end

function test_RL()
    sizes = [11, 15, 20, 50, 100]
    n_groups = [1, 1, 2, 5, 10]
    n_samples = 10
    results = zeros(length(sizes), 3)
    for i in 1:length(sizes)
        @info "Testing n = $(sizes[i]) / DPS: n_groups = $(n_groups[i]) /  RL: n_samples = $n_samples"
        results[i, :] .= solve_tspd_RL_test(sizes[i], n_groups[i], n_samples)
    end
end

