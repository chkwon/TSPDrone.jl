
import os
import numpy as np
import time
from concorde.tsp import TSPSolver
# This requires pyconcorde: https://github.com/jvkersch/pyconcorde
# Install:  python3 -m pip install git+git://github.com/jvkersch/pyconcorde.git


def _distance_matrices(x, y, speed_of_truck, speed_of_drone):
    n_nodes = len(x)
    assert len(x) == len(y)

    dist = np.zeros((n_nodes, n_nodes), dtype=float)
    for i in range(n_nodes):
        for j in range(n_nodes):
            dist[i, j] = np.sqrt( (x[i]-x[j])**2 + (y[i]-y[j])**2 )

    Ct = speed_of_truck * dist
    Cd = speed_of_drone * dist
    return Ct, Cd

def distance_matrices(x, y, speed_of_truck, speed_of_drone):
    xx = x.copy()
    yy = y.copy()
    xx.append(x[0])
    yy.append(y[0])
    Ct, Cd = _distance_matrices(xx, yy, speed_of_truck, speed_of_drone)
    return  Ct, Cd

def read_data(filename, n):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    inst_dir = f"../ExactSolutions/instances/n{n}/"
    filepath = os.path.join(dir_path, inst_dir, filename)

    f = open(filepath, "r")
    f.readline()
    speed_of_truck = float(f.readline())
    f.readline()
    speed_of_drone = float(f.readline())
    f.readline()
    n_nodes = int(f.readline())
    f.readline()
    depot_x, depot_y, _ = f.readline().split()
    depot_x = float(depot_x)
    depot_y = float(depot_y)
    f.readline()
    x = [depot_x]
    y = [depot_y]
    for i in range(n_nodes-1):
        cx, cy, _ = f.readline().split()
        x.append(float(cx))
        y.append(float(cy)) 
    return x, y, speed_of_truck, speed_of_drone




def find_tsp_tour(x, y):
    tsp_solver = TSPSolver.from_data(x, y, norm="GEOM")
    tsp_sol = tsp_solver.solve()
    tsp_tour = tsp_sol.tour.tolist()
    return tsp_tour


def exact_partitioning(initial_tour, n_nodes, Ct, Cd):
    r = initial_tour
    T = np.ones((n_nodes+1, n_nodes+1)) * np.inf
    M = np.ones((n_nodes+1, n_nodes+1), dtype=int) * (-2)
    for i in range(n_nodes):
        for j in range(i + 1, n_nodes+1):
            if j == i + 1:
                T[i, j] = Ct[r[i], r[j] ]
                M[r[i], r[j]] = -1
            
            for k in range(i+1, j):
                Tk1 = Cd[r[i], r[k]] + Cd[r[k], r[j]]
                Tk2 = sum([Ct[r[l], r[l+1]] for l in range(i, k-1)]) \
                    + Ct[r[k-1], r[k+1]] \
                    + sum([Ct[r[l], r[l+1]] for l in range(k+1, j)])
                Tk = max(Tk1, Tk2)
                if Tk < T[i, j]:
                    T[i, j] = Tk
                    M[r[i], r[j]] = r[k]

    V = np.zeros(n_nodes+1)
    P = np.ones(n_nodes+1, dtype=int) * (-1)

    V[0] = 0 
    for i in range(1, n_nodes+1):
        VV = [V[k] + T[k, i] for k in range(i)]
        V[i] = min(VV)
        P[i] = r[VV.index(V[i])]

    final_time = V[-1]
    # return final_time, V, P, M, initial_tour
    return final_time

def two_point_move(tour, i, j):
    tmp = tour.copy()
    tmp[j] = tour[i]
    tmp[i] = tour[j]
    return tmp

def one_point_move(tour, i, j):
    tmp = tour.copy()
    del tmp[i]
    tmp.insert(j, tour[i])
    return tmp    

def two_opt_move(tour, i, j):
    tmp = tour.copy()
    tmp[i:j+1] = tour[j:i-1:-1]
    return tmp        


# Main function to call
def tsp_ep_all(x_coordinates, y_coordinates, truck_cost_factor, drone_cost_factor):
    """
    Runs `TSP-ep-all` heuristic algorithm of Agatz et al.

    `x_coordinates[0]`, `y_coordinates[0]`: the coordinates of the depot, then followed by all customer location coordinates
    `truck_cost_factor`: as defined in Agatz et al. instances
    `drone_cost_factor`: as defined in Agatz et al. instances
    """

    Ct, Cd = distance_matrices(x_coordinates, y_coordinates, truck_cost_factor, drone_cost_factor)
    n_nodes = len(x_coordinates)
    n1, n2 = Ct.shape
    assert n_nodes + 1 == n1
    assert Ct.shape == Cd.shape

    tsp_tour = find_tsp_tour(x_coordinates, y_coordinates)
    tsp_tour.append(n_nodes) # adding a dummy node for the returning depot 

    improved = True
    best_obj = np.inf
    best_tour = tsp_tour.copy()
    while improved:
        improved = False
        cur_best_obj = best_obj
        cur_best_tour = best_tour.copy()

        for i in range(1, n_nodes):
            # 2p (two-point move)
            for j in range(i+1, n_nodes):
                new_tour = two_point_move(best_tour, i, j)
                ep_time = exact_partitioning(new_tour, n_nodes, Ct, Cd)
                if ep_time < cur_best_obj:
                    cur_best_tour = new_tour.copy()
                    cur_best_obj = ep_time
                    improved = True 
            
            # 1p (one-point move)
            for j in range(1, n_nodes):
                new_tour = one_point_move(best_tour, i, j)
                ep_time = exact_partitioning(new_tour, n_nodes, Ct, Cd)
                if ep_time < cur_best_obj:
                    cur_best_tour = new_tour.copy()
                    cur_best_obj = ep_time
                    improved = True     

            # 2-opt (two-opt move)
            for j in range(i+1, n_nodes-1):
                new_tour = two_opt_move(best_tour, i, j)
                ep_time = exact_partitioning(new_tour, n_nodes, Ct, Cd)
                if ep_time < cur_best_obj:
                    cur_best_tour = new_tour.copy()
                    cur_best_obj = ep_time
                    improved = True     

        if improved:
            best_obj = cur_best_obj
            best_tour = cur_best_tour

    return best_obj






# Test for n11 instances

def tsp_ep_all_agatz(filename, n):
    tsp_x, tsp_y, truck_cost_factor, drone_cost_factor = read_data(filename, n)
    return tsp_ep_all(tsp_x, tsp_y, truck_cost_factor, drone_cost_factor)


if __name__ == "__main__":
    exact_opt_obj = {}
    # n11
    exact_opt_obj[11] = np.array([
        221.18876576478925,
        205.76050725572097,
        192.96313461174037,
        241.25592289521398,
        248.1379946498235,
        217.68894293889753,
        237.34013623078425,
        214.76536428997835,
        256.33972821148967,
        227.90300661076967
    ])

    # n15
    exact_opt_obj[15] = np.array([
        260.19649903254805,  
        279.8748849328301,
        278.2501294150187,
        227.4927030824479,
        233.0058389136502,
        252.9062262214419,
        288.5045658852292,
        277.4202725739683,
        280.23702686030765,
        271.52779790946164 
    ])

    def test_agatz(n):
        print("Testing for n =", n)

        obj_val = []
        for i in range(1, 11):
            filename = f"uniform-{i}-n{n}.txt"
            val = tsp_ep_all_agatz(filename, n)
            # print(val)
            obj_val.append(val)

        print(obj_val)
        obj_val = np.array(obj_val)
        gap = (obj_val - exact_opt_obj[n]) / exact_opt_obj[n] * 100
        print("average gap = ", np.mean(gap), " %")
        print("maximum gap = ", np.max(gap), " %")

    print("-----------------------------")
    t0 = time.time()
    test_agatz(11)
    t1 = time.time()
    print("average time = ", (t1-t0) / 10, " seconds")
    print("-----------------------------")
    t0 = time.time()
    # test_agatz(15)
    t1 = time.time()
    print("average time = ", (t1-t0) / 10, " seconds")
    print("-----------------------------")

