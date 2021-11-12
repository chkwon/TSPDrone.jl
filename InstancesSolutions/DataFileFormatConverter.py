import numpy as np 
fname = 'RL-TSP-len-100-n_nodes-100.txt'
val_data = np.loadtxt(fname,delimiter=' ')
n_nodes =100
batch_size = 100
val_data = val_data.reshape(-1, n_nodes,3)

data = np.zeros([100, n_nodes, 2])

data[:, 0, :] = val_data[:, n_nodes-1, :2]

data[:, 1:n_nodes, :] = val_data[:, :n_nodes-1, :2]


np.savetxt("Concorde-TSP-len-{}-n_nodes-{}.txt".format(batch_size, n_nodes), data.reshape(-1, n_nodes*2))


