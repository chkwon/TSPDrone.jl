import numpy as np 
import os 
import torch 
import random
from options import ParseParams
from env_no_comb import Env 
from nnets import Actor, Critic 
from agent import A2CAgent 
from AttentionModel import AttentionModel 
import time

def check(x, y, n_samples, dev):
    print(x)
    return  0

def main(x, y, n_samples, dev, decode_len):
    args = ParseParams()   
    random_seed = args['random_seed']
    if random_seed is not None and random_seed > 0:
        print("# Set random seed to %d" % random_seed)
    np.random.seed(random_seed)
    random.seed(random_seed)
    torch.manual_seed(random_seed)
    max_epochs = args['n_train']
    device = torch.device(dev) 
    save_path = args['save_path']
    data = np.concatenate((np.array(x)[:, None], np.array(y)[:, None], np.ones([len(x), 1])), 1)[None, :, :]
    data[0, len(x)-1, 2] =0
    env = Env(args, data)
    actor = Actor(args['hidden_dim'], mode='drone', dev=dev)
    critic = Critic(args['hidden_dim'], dev)
    n_nodes = len(x)
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    else:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        # path = "src/PythonRL/" + save_path +'n' +str(n_nodes) + '/best_model_actor_truck_params.pkl'
        path = os.path.join(dir_path, save_path, 'n' + str(n_nodes), 'best_model_actor_truck_params.pkl')

        if os.path.exists(path):
            actor.load_state_dict(torch.load(path, map_location='cpu'))
            # path = "src/PythonRL/" + save_path +'n' +str(n_nodes) + '/best_model_critic_params.pkl'
            path = os.path.join(dir_path, save_path, 'n' + str(n_nodes), 'best_model_critic_params.pkl')

            critic.load_state_dict(torch.load(path, map_location='cpu'))
            print("Succesfully loaded keys")
        else: 
            raise Exception("No trained neural net for n_nodes = " + str(n_nodes))
    
    agent = A2CAgent(actor, critic, args, env, dev, decode_len)
    if n_samples ==1:
        obj, route_t, route_d = agent.test(data)
    else:
        obj, route_t, route_d = agent.sampling_batch(n_samples, data)
        
    return obj, route_t, route_d
       

#x = [2, 3, 4, 7, 1, 3, 5, 6]
#y = [4, 5, 6, 1, 4, 5, 7, 9]
#dev = "cpu"
#n_samples = 3
#main(x, y, n_samples, dev)


