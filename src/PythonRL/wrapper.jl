

include("test_RL.jl")

function solve_tspd_RL(x::Vector{Float64}, y::Vector{Float64}; n_samples=1, device="cpu")
    supported_sizes = [11, 15, 20, 50, 100]
    n_nodes = length(x)
    if !in(n_nodes, supported_sizes)
        error("Not supported size: n = $(n_nodes). RL support n ∈ $supported_sizes only.")
    end
    # input x, y: depot is x[1], y[1]

    # in Python RL model, depot is the last element
    xx = [x[2:end]; x[1]]
    yy = [y[2:end]; y[1]]

    # table = Dict([11 => 25, 20 => 35, 50 => 85, 100 => 180])
    # decode_len = table[n_nodes]
    decode_len = round(Int, n_nodes * 1.8)

    pushfirst!(PyVector(pyimport("sys")."path"), @__DIR__)

    py"""
    def solve_tspd(x, y, n_samples, device, decode_len):
        import main
        return main.main(x, y, n_samples, device, decode_len)
    """
    
    obj, route_t_, route_d_ = py"solve_tspd"(xx, yy, n_samples, device, decode_len)

    # +1 for python-julia indexing difference
    # another +1 for depot numbering difference
    route_t = Int.(route_t_) .+ 2
    route_d = Int.(route_d_) .+ 2

    # remove same nodes
    unique!(route_t)
    unique!(route_d)
    # add the origin depot 
    pushfirst!(route_t, 1)
    pushfirst!(route_d, 1)

    return obj, route_t, route_d
end


