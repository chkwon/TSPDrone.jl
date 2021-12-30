include("../src/main.jl")

n = 11
x = rand(n) .* 100
y = rand(n) .* 100
x[1] = rand() # depot
y[1] = rand() # depot

# x = [0.8172268241831585, 73, 61, 44, 74, 23, 9, 51, 15, 9, 56]
# y = [0.6284331187597952, 52, 46, 63, 15, 23, 80, 65, 97, 74, 84]


obj1, tr1, dr1 = solve_tspd(x, y, 1.0, 0.5; n_groups=1)
obj2, tr2, dr2 = solve_tspd_RL(x, y; n_samples=10)

@show obj1, obj2
@show tr1, tr2 
@show dr1, dr2 

println("Done.")