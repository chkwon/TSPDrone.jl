import numpy as np 
import os
import numpy as np
import time
from concorde.tsp import TSPSolver
import copy

class dataGen(object):
    def __init__(self, batch_size, n_nodes, v_t, v_d):
        self.n_nodes = n_nodes 
        self.batch_size = batch_size 
        
        # run this to validate the results with the original implementation 
    #    fname = 'uniform-n11.txt'
    #    data = np.loadtxt(fname,delimiter=' ', dtype=np.float64)
    #    data = data.reshape(-1, 11,4)
    #    data2 = copy.deepcopy(data)
    #    for i in range(10):
    #        data2[i, 0, :] = data[i, 10, :]
     #       for j in range(1, 11):
      #          data2[i, j, :] = data[i, j-1, :]
      #  self.input_pnt = data2[:, :, :2]
        # creates dataset and saves for RL and concord 
        # In RL depot is the last node, in concord it is the first node 
        self.input_pnt = self.create_test_dataset(batch_size, n_nodes)
        self.v_t = v_t
        self.v_d = v_d 
        
    def forward(self):
        self.dist_mat = np.zeros([self.batch_size, self.n_nodes+1, self.n_nodes+1])
        for i in range(self.n_nodes):
            for j in range(i+1, self.n_nodes):
                self.dist_mat[:, i, j] = ((self.input_pnt[:, i, 0] - self.input_pnt[:, j, 0])**2 + (self.input_pnt[:, i, 1] - self.input_pnt[:, j, 1])**2)**0.5
                self.dist_mat[:, j, i] =  self.dist_mat[:, i, j]
        self.dist_mat[:, self.n_nodes, :] = self.dist_mat[:, 0, :]
        self.dist_mat[:, :, self.n_nodes] = self.dist_mat[:, :, 0]
        self.truck_mat = self.dist_mat*v_t
        self.drone_mat = self.dist_mat*v_d
        
        return self.input_pnt, self.truck_mat, self.drone_mat
    
    def create_test_dataset(self, batch_size, n_nodes):
        x = np.random.choice(100, size=(batch_size, n_nodes-1))
        y = np.random.choice(100, size=(batch_size, n_nodes-1))
        depot = np.random.uniform(0, 1, size=(batch_size, 1, 2))
        data = np.concatenate([np.expand_dims(x, 2), np.expand_dims(y, 2)], 2)
        #make the first  node depot 
        input_data =  np.concatenate([depot, data], 1)
        input_data2 =  np.concatenate([data, depot], 1)
        np.savetxt("Concorde-TSP-len-{}-n_nodes-{}.txt".format(batch_size, n_nodes), input_data.reshape(-1, n_nodes*2))
        np.savetxt("RL-TSP-len-{}-n_nodes-{}.txt".format(batch_size, n_nodes), input_data2.reshape(-1, n_nodes*2))

        return input_data

# Main function to call
def tsp_ep_all(x, y, Ct, Cd):
    """
    Runs `TSP-ep-all` heuristic algorithm of Agatz et al.

    `x_coordinates[0]`, `y_coordinates[0]`: the coordinates of the depot, then followed by all customer location coordinates
    `speed_truck`: as defined in Agatz et al. instances
    `speed_drone`: as defined in Agatz et al. instances
    """

    
    n_nodes = len(x)
    n1, n2 = Ct.shape
    assert n_nodes + 1 == n1
    assert Ct.shape == Cd.shape

    tsp_tour = find_tsp_tour(x, y)
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

# speed of truck v_t, speed of drone v_d 
v_t = 1
v_d = 0.5
batch_size = 100
n_nodes = 50


if __name__ == "__main__":
    datagen = dataGen(batch_size, n_nodes, v_t, v_d)
    input_pnt, truck_mat, drone_mat = datagen.forward()
    obj_vals = []
    t_begin = time.time()
    for i in range(batch_size):
        x, y = list(input_pnt[i, :, 0]), list(input_pnt[i, :, 1])
        Ct, Cd = truck_mat[i], drone_mat[i]
        obj = tsp_ep_all(x, y, Ct, Cd)
        obj_vals.append(obj)
    t_end = time.time() - t_begin
    t_aver = t_end/batch_size
    t = [t_end, t_aver]
    np.savetxt("Agatz-heuristic-len-{}-n_nodes-{}.txt".format(batch_size, n_nodes), obj_vals)
    np.savetxt("Agatz-heuristic-runtime-len-{}-n_nodes-{}.txt".format(batch_size, n_nodes), t)