import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import os 
import copy
import time 
import random
import numpy as np 
from utils import printOut

class A2CAgent(object):
    
    def __init__(self, actor, critic, args, env, dev, decode_len):
        self.actor = actor
        self.critic = critic 
        self.args = args 
        self.env = env 
        self.device = dev
        self.decode_len = decode_len
     
        dir_path = os.path.dirname(os.path.realpath(__file__))

        # out_file = open(os.path.join(dir_path, args['log_dir'], 'results.txt'),'w+') 
        # self.prt = printOut(out_file,args['stdout_print'])
        # print("agent is initialized")
        

    def test(self, data):
        args = self.args 
        env = self.env 
        actor  = self.actor
        device = self.device
     
        # prt = self.prt 
        n = 0
        actor.eval()
        
        env.input_data = data 
   #     print("test_data: ", data[n])
        state, avail_actions = env.reset()
   #     prt.print_out("test started")
   #     prt.print_out("{}: {}".format("initial state",state[n]))
   #     prt.print_out("{}: {}".format("avail_actions",avail_actions[n]))
        
       
        time_vec_truck = np.zeros([env.batch_size, 2])
        time_vec_drone = np.zeros([env.batch_size, 3])
        sols = []
        costs = []
        with torch.no_grad():
            data = torch.from_numpy(data[:, :, :2].astype(np.float32)).to(device)
            # [b_s, hidden_dim, n_nodes]
            static_hidden = actor.emd_stat(data).permute(0, 2, 1)
           
            
          
            # lstm initial states 
            hx = torch.zeros(1, env.batch_size, args['hidden_dim']).to(device)
            cx = torch.zeros(1, env.batch_size, args['hidden_dim']).to(device)
            last_hh = (hx, cx)
       
            # prepare input 
            ter = np.zeros(env.batch_size).astype(np.float32)
            decoder_input = static_hidden[:, :, env.n_nodes-1].unsqueeze(2)
            time_step = 0
            while time_step < self.decode_len:
                terminated = torch.from_numpy(ter.astype(np.float32)).to(device)
                for j in range(2):
                    # truck takes action 
                    if j == 0:
                   #     prt.print_out("{}: {}".format("avail_actions_truck", avail_actions[n]))
                        avail_actions_truck = torch.from_numpy(avail_actions[:, :, 0].reshape([env.batch_size, env.n_nodes]).astype(np.float32)).to(device)
                        dynamic_truck = torch.from_numpy(np.expand_dims(state[:, :, 0], 2)).to(device)
                        idx_truck, prob, logp, last_hh = actor.forward(static_hidden, dynamic_truck, decoder_input, last_hh, 
                                                                     terminated, avail_actions_truck)
                        b_s = np.where(np.logical_and(avail_actions[:, :, 1].sum(axis=1)>1, env.sortie==0))[0]
                        avail_actions[b_s, idx_truck[b_s].cpu(), 1] = 0
                        avail_actions_drone = torch.from_numpy(avail_actions[:, :, 1].reshape([env.batch_size, env.n_nodes]).astype(np.float32)).to(device)
                        idx = idx_truck 
                 #       prt.print_out("{}: {}".format("idx_truck", idx_truck[n]))
                   #     prt.print_out("{}: {}".format("avail_actions_drone", avail_actions_drone[n]))
                       
                    else:
                        dynamic_drone = torch.from_numpy(np.expand_dims(state[:, :, 1], 2)).to(device)
                        idx_drone, prob, logp, last_hh = actor.forward(static_hidden, dynamic_drone, decoder_input, last_hh, 
                                                                     terminated, avail_actions_drone)
                        idx = idx_drone 
                 #       prt.print_out("{}: {}".format("idx_drone", idx_drone[n]))
                       
                    
                 #   prt.print_out("{}: {}".format("idx", idx[n]))
                 #   prt.print_out("{}: {}".format("j", j))
                    decoder_input =  torch.gather(static_hidden, 2, idx.view(-1, 1, 1).expand(env.batch_size, args['hidden_dim'], 1)).detach()
                
                state, avail_actions, ter, time_vec_truck, time_vec_drone = env.step(idx_truck.cpu().numpy(), idx_drone.cpu().numpy(), time_vec_truck, time_vec_drone, ter)
                time_step += 1
                sols.append([idx_truck[n], idx_drone[n]])
                costs.append(env.time_step[n])
    #             prt.print_out("{}: {}".format("idx_truck", idx_truck[n]))
    #             prt.print_out("{}: {}".format("idx_drone", idx_drone[n]))
    #  #           prt.print_out("{}: {}".format("comb check", env.combined_check[n]))
    #             prt.print_out("{}: {}".format("time_step", env.time_step[n]))
    #             prt.print_out("{}: {}".format("current_time",env.current_time[n]))
    #             prt.print_out("{}: {}".format("truck_loc", env.truck_loc[n]))
    #             prt.print_out("{}: {}".format("drone_loc", env.drone_loc[n]))
    #  #           prt.print_out("{}: {}".format("comb nodes", env.combined_nodes[n]))
    #             prt.print_out("{}: {}".format("time_vec_truck", time_vec_truck[n]))
    #             prt.print_out("{}: {}".format("time_vec_drone", time_vec_drone[n]))
    #             prt.print_out("{}: {}".format("state", env.state[n]))
    #     #        prt.print_out("{}: {}".format("p", env.p[n]))
    #             prt.print_out("{}: {}".format("avail_actions", avail_actions[n]))
    #             prt.print_out("{}: {}".format("terminated", terminated[n]))
    #             prt.print_out("{}: {}".format("sortie", env.sortie[n]))
    #             prt.print_out("{}: {}".format("returned", env.returned[n]))
            
        R = copy.copy(env.current_time)
        costs.append(env.current_time[n])
        # print("finished: ", sum(terminated))
       
        sols = np.array(sols)
        
    #    fname = 'test_results-{}-len-{}.txt'.format(args['test_size'], 
                    #                                           args['n_nodes'])
    #    fname = 'results/' + fname
    #    np.savetxt(fname, R)
    #    np.savetxt('results/route_t.txt', sols[:, 0])
    #    np.savetxt('results/route_d.txt', sols[:, 1])
     #   np.savetxt('results/costs.txt', costs)
            
      
        return R, sols[:, 0], sols[:, 1]
    
    

    def sampling_batch(self, sample_size, data):
        device = self.device
        val_size = data.shape[0]
        
        #best_sols = np.zeros([val_size, self.args['decode_len'], 2])
        sols = np.zeros([sample_size, self.decode_len, 2])
        args = self.args 
        env = self.env 
        actor  = self.actor
     
        # prt = self.prt 
        n = 0
        actor.eval()
        actor.set_sample_mode(True)
       
        times = []
        initial_t = time.time()
       
       
        data_list = [np.expand_dims(data[i, ...], axis=0) for i in range(data.shape[0])]
        best_rewards_list = []
        for d in data_list:
            # batched sampling
            data = np.repeat(d, sample_size, axis=0)
            env.input_data = data 
            
            state, avail_actions = env.reset()
            
            #time_vec_truck = np.zeros([env.batch_size, 2])
            #time_vec_drone = np.zeros([env.batch_size, 3])
            time_vec_truck = np.zeros([sample_size, 2])
            time_vec_drone = np.zeros([sample_size, 3])
            with torch.no_grad():
                data = torch.from_numpy(data[:, :, :2].astype(np.float32)).to(device)
                # [b_s, hidden_dim, n_nodes]
                static_hidden = actor.emd_stat(data).permute(0, 2, 1)
            
                # lstm initial states 
                #hx = torch.zeros(1, env.batch_size, args['hidden_dim']).to(device)
                #cx = torch.zeros(1, env.batch_size, args['hidden_dim']).to(device)
                hx = torch.zeros(1, sample_size, args['hidden_dim']).to(device)
                cx = torch.zeros(1, sample_size, args['hidden_dim']).to(device)
                last_hh = (hx, cx)
        
                # prepare input 
                #ter = np.zeros(env.batch_size).astype(np.float32)
                ter = np.zeros(sample_size).astype(np.float32)
                decoder_input = static_hidden[:, :, env.n_nodes-1].unsqueeze(2)
                time_step = 0
                while time_step < self.decode_len:
                    terminated = torch.from_numpy(ter).to(device)
                    for j in range(2):
                        # truck takes action 
                        if j == 0:
                            #     prt.print_out("{}: {}".format("avail_actions_truck", avail_actions[n]))
                            #avail_actions_truck = torch.from_numpy(avail_actions[:, :, 0].reshape([env.batch_size, env.n_nodes]).astype(np.float32)).to(device)
                            avail_actions_truck = torch.from_numpy(avail_actions[:, :, 0].reshape([sample_size, env.n_nodes]).astype(np.float32)).to(device)
                            dynamic_truck = torch.from_numpy(np.expand_dims(state[:, :, 0], 2)).to(device)
                            idx_truck, prob, logp, last_hh = actor.forward(static_hidden, dynamic_truck, decoder_input, last_hh, 
                                                                        terminated, avail_actions_truck)
                            b_s = np.where(np.logical_and(avail_actions[:, :, 1].sum(axis=1)>1, env.sortie==0))[0]
                            avail_actions[b_s, idx_truck[b_s].cpu(), 1] = 0
                            #avail_actions_drone = torch.from_numpy(avail_actions[:, :, 1].reshape([env.batch_size, env.n_nodes]).astype(np.float32)).to(device)
                            avail_actions_drone = torch.from_numpy(avail_actions[:, :, 1].reshape([sample_size, env.n_nodes]).astype(np.float32)).to(device)
                            idx = idx_truck 
                            #       prt.print_out("{}: {}".format("idx_truck", idx_truck[n]))
                            #     prt.print_out("{}: {}".format("avail_actions_drone", avail_actions_drone[n]))
                        
                        else:
                            dynamic_drone = torch.from_numpy(np.expand_dims(state[:, :, 1], 2)).to(device)
                            idx_drone, prob, logp, last_hh = actor.forward(static_hidden, dynamic_drone, decoder_input, last_hh, 
                                                                        terminated, avail_actions_drone)
                            idx = idx_drone 
                            #       prt.print_out("{}: {}".format("idx_drone", idx_drone[n]))

                        #decoder_input =  torch.gather(static_hidden, 2, idx.view(-1, 1, 1).expand(env.batch_size, args['hidden_dim'], 1)).detach()
                        decoder_input =  torch.gather(static_hidden, 2, idx.view(-1, 1, 1).expand(sample_size, args['hidden_dim'], 1)).detach()
                
                    state, avail_actions, ter, time_vec_truck, time_vec_drone = env.step(idx_truck.cpu().numpy(), idx_drone.cpu().numpy(), time_vec_truck, time_vec_drone, ter)
                    sols[:, time_step, 0] = idx_truck.cpu().numpy()
                    sols[:, time_step, 1] = idx_drone.cpu().numpy()
                    time_step += 1
                    
                    
            
            R = copy.copy(env.current_time)
            # print("R: ", R)
            # print('sols', sols)
            best_rewards = R.min(axis=0)
            idx = np.argmin(R, axis=0)
            best_sols = sols[idx]

        return best_rewards, best_sols[:, 0], best_sols[:, 1]
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    