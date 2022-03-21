using PyCall
using Printf

function install_python_dependencies()
    Python_PACKAGES = ["numpy", "scipy", "torch"]

    try
        pip = pyimport("pip")
    catch
        get_pip = joinpath(dirname(@__FILE__), "get-pip.py")
        download("https://bootstrap.pypa.io/get-pip.py", get_pip)
        run(`$(PyCall.python) $get_pip --user`)
    end

    pip = pyimport("pip")
    args = String[]
    if haskey(ENV, "http_proxy")
        push!(args, "--proxy")
        push!(args, ENV["http_proxy"])
    end
    push!(args, "install")
    push!(args, "--user")
    append!(args, Python_PACKAGES)

    pip.main(args)
end


function solve_tspd_RL_test_corner_depot(n_nodes, n_groups, n_samples)
    n_runs = 20
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
    @printf("mean(obj_DPS) \t mean(obj_RL) \t pct_gap\n")
    @printf("------------- \t ------------ \t -------\n")
    @printf("%13.2f \t %12.2f \t %5.2f %%\n", mean(obj_DPS), mean(obj_RL), pct_gap)

    @test pct_gap < 0.05
    return mean(obj_DPS), mean(obj_RL), pct_gap
end

function solve_tspd_RL_test_uniform_depot(n_nodes, n_groups, n_samples)
    n_runs = 20
    obj_DPS = zeros(n_runs)
    obj_RL = zeros(n_runs)
    for i in 1:n_runs
        x = rand(n_nodes) .* 100
        y = rand(n_nodes) .* 100
        x[1] = rand() * 100 # depot
        y[1] = rand() * 100 # depot
        
        # x = [0.8172268241831585, 73, 61, 44, 74, 23, 9, 51, 15, 9, 56]
        # y = [0.6284331187597952, 52, 46, 63, 15, 23, 80, 65, 97, 74, 84]
        
        obj_DPS[i], _, _ = solve_tspd(x, y, 1.0, 0.5; n_groups=n_groups)
        obj_RL[i], _, _ = solve_tspd_RL(x, y; n_samples=n_samples)
    end
    pct_gap = (mean(obj_RL) - mean(obj_DPS)) / mean(obj_DPS)
    @printf("mean(obj_DPS) \t mean(obj_RL) \t pct_gap\n")
    @printf("------------- \t ------------ \t -------\n")
    @printf("%13.2f \t %12.2f \t %5.2f %%\n", mean(obj_DPS), mean(obj_RL), pct_gap)

    @test pct_gap < 0.05
    return mean(obj_DPS), mean(obj_RL), pct_gap
end

function test_RL()
    install_python_dependencies()

    sizes = [11, 15, 20, 50, 100]
    n_groups = [1, 1, 2, 5, 10]
    n_samples = 10
    results = zeros(length(sizes), 3)
    for i in 1:length(sizes)
        @info "Testing Corner Depot: n = $(sizes[i]) / DPS: n_groups = $(n_groups[i]) /  RL: n_samples = $n_samples"
        results[i, :] .= solve_tspd_RL_test_corner_depot(sizes[i], n_groups[i], n_samples)
    end

    for i in 3:length(sizes)
        @info "Testing Uniform Depot: n = $(sizes[i]) / DPS: n_groups = $(n_groups[i]) /  RL: n_samples = $n_samples"
        results[i, :] .= solve_tspd_RL_test_uniform_depot(sizes[i], n_groups[i], n_samples)
    end    
end

