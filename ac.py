from math import ceil, floor, exp
from networkx.classes.digraph import DiGraph
from networkx import shortest_path, shortest_path_length

import numpy as np

import random

import pickle

import time


class actor_critic(object):
    def __init__(self, crm, data_bound, quant_size, sample_time, uncertainty_prob=[0,0], clambda=0.98, gamma=1, beta=1, D=1, theta0=None):
        self.crm = crm    # crm = 'combined roadmap'
        self.data_bound = data_bound
        self.quant_size = quant_size
        self.data_set = [k*quant_size for k in range(int(floor(data_bound/quant_size))+1)]
        self.sample_time = sample_time
        self.uncertainty = uncertainty_prob
        self.arg_crm = None
        self.shortest_path = None
        # for critic 
        self.clambda = clambda
        self.gamma = gamma
        self.z = np.array([0.0, 0.0])
        self.b = np.array([0.0, 0.0])
        self.r = np.array([0.0, 0.0])
        self.A = np.array([[1.0,0.0],[0.0,1.0]])
        # for actor
        if theta0:
            self.theta = np.array(theta0)
        else:
            self.theta = np.array([1.0, 1.0])
        self.d_theta = np.array([0.0, 0.0])
        self.D = D
        self.beta = beta
        print '---- actor_critic learner initialized: %d waypoints and %d data cells----' %(len(self.crm.nodes()), len(self.data_set))

    def set_init_goal(self, init, goal):
        self.init = init
        self.goal = goal

    def set_theta(self, theta):
        self.theta = np.array(theta)
        
    def cost(self, sa_f):
        s_f, a_f = sa_f
        if s_f == self.goal:
            return 0
        else:
            return 1

    def static_dijkstra(self):
        print '||||||||||||static arg_crm dijkstra starts||||||||||||'
        start = time.time()
        #----------
        if not self.arg_crm:
            self.construct_arg_crm()
        #----------
        deter_path = shortest_path(self.arg_crm, self.init, self.goal)
        print '----shorest_path from arg_crm derived within %.2f s: length %d' %(time.time()-start, len(deter_path))
        self.shortest_path = list(deter_path)
        # pickle.dump(deter_path, open('data/static_arg_crm_dijkstra.p', "wb"))
        # print('----saved as data/static_arg_crm_dijkstra.p----')
        return deter_path
    
    def construct_arg_crm(self):
        arg_crm = DiGraph()
        for wp_f in self.crm.nodes_iter():
            for d_f in iter(self.data_set):
                s_f = (wp_f, d_f)
                arg_crm.add_node(s_f)
                for wp_t in self.crm.neighbors(wp_f):
                    d_evolve = d_f-self.crm.edge[wp_f][wp_t]['rate']*self.sample_time-2*self.quant_size
                    d_t = [d for d in self.data_set if d>= d_evolve][0]
                    s_t = (wp_t, d_t)
                    arg_crm.add_edge(s_f, s_t, weight=1.0)
        print '---static argumented crm constructed: %d nodes, %d edges----' %(len(arg_crm.nodes()), len(arg_crm.edges()))
        self.arg_crm = arg_crm
        
    def uncertain_rate(self, exp_rate):
        # triangular distribution as an example 
        actual_rate = np.random.triangular(exp_rate*self.uncertainty[0], exp_rate, exp_rate*self.uncertainty[1])+(self.quant_size/self.sample_time)
        return actual_rate

    def successor_prob(self, sa_f, mode='model'):
        s_f, a_f = sa_f
        if mode == 'model':
            s_ts = [nb for nb in self.arg_crm.neighbors(s_f) if nb[0] == a_f]
            prob_ts = [self.arg_crm.edge[s_f][nb]['weight'] for nb in s_ts]
            uni_prob_ts = [w/sum(prob_ts) for w in prob_ts]
        elif mode == 'experiment':
            prob_ts_approx = dict()
            for k in range(50):
                s_t, actual_rate = self.successor_deter(sa_f, mode)
                self.update_arg_crm(sa_f, s_t)
                if s_t not in prob_ts_approx:
                    prob_ts_approx[s_t] = 1
                else:
                    prob_ts_approx[s_t] += 1
            s_ts = list(prob_ts_approx.keys())
            prob_ts = [prob_ts_approx[s_t] for s_t in s_ts]
            uni_prob_ts = [w/sum(prob_ts) for w in prob_ts]
        return s_ts, uni_prob_ts
    
    def successor_deter(self, sa_f, mode='experiment'):
        s_f, a_f = sa_f
        # uncertainty in motion or rate comes here
        if mode == 'experiment':
            exp_rate = self.crm.edge[s_f[0]][a_f]['rate']
            actual_rate = self.uncertain_rate(exp_rate)
            d_evolve = s_f[1]-actual_rate*self.sample_time
            d_next = [d for d in self.data_set if d>= d_evolve][0]
            s_t = (a_f, d_next)
        elif mode == 'model':
            if not self.arg_crm:
                self.construct_arg_crm()
            s_ts, uni_prob_ts = self.successor_prob(sa_f, mode='model')
            prob_list = self.accumulate(uni_prob_ts)
            prob = random.random()
            j = [j for j,p in enumerate(prob_list) if p>prob][0]
            s_t = s_ts[j]
            actual_rate = (s_t[1] - s_f[1])/self.sample_time
        return s_t, actual_rate

    def update_arg_crm(self, sa_f, s_t):
        s_f, a_f = sa_f
        if s_t not in self.arg_crm.neighbors(s_f):
            self.arg_crm.add_edge(s_f, s_t, weight=1)
        else:
            self.arg_crm.edge[s_f][s_t]['weight'] += 1

    def train_arg_crm(self):
        if not self.arg_crm:
            self.construct_arg_crm()
        for s_f in self.arg_crm.nodes():
            for a_f in self.crm.neighbors(s_f[0]):
                sa_f = (s_f, a_f)
                s_t, actual_rate = self.successor_deter(sa_f, mode='experiment')
                self.update_arg_crm(sa_f, s_t)

    def static_dijkstra(self):
        print '||||||||||||static arg_crm dijkstra starts||||||||||||'
        start = time.time()
        #----------
        if not self.arg_crm:
            self.construct_arg_crm()
        #----------
        deter_path = shortest_path(self.arg_crm, self.init, self.goal)
        print '----shorest_path from arg_crm derived within %.2f s: length %d' %(time.time()-start, len(deter_path))
        self.shortest_path = list(deter_path)
        pickle.dump(deter_path, open('data/new_case_static_arg_crm_dijkstra.p', "wb"))
        print '----saved as data/new_case_static_arg_crm_dijkstra.p----'
        return deter_path            

    def compute_parameter(self, sa_f, mode='model'):
        #print('**********')
        #print('compute_paramter: sa_f, %s' %str(sa_f))
        s_f, a_f = sa_f
        s_ts, uni_prob_ts = self.successor_prob(sa_f, mode)
        #print('s_ts, %s, uni_prob_ts: %s' %(str(s_ts), str(uni_prob_ts)))
        sum_distance_dif = 0
        sum_data_dif = 0
        for k, s_t in enumerate(s_ts):
            distance_dif = shortest_path_length(self.crm, s_t[0], self.goal[0])
            sum_distance_dif += uni_prob_ts[k]*distance_dif
            # d_tt = [s_tt[1] for s_tt in self.arg_crm.neighbors(s_t)]
            # d_tt_ave = sum(d_tt)/len(d_tt)
            # data_dif = abs(self.goal[1]-d_tt_ave)
            data_dif = abs(self.goal[1]- s_t[1])
            sum_data_dif += uni_prob_ts[k]*data_dif
            #print('s_t: %s, distance_self: %f, data_self: %f' %(str(s_t), distance_dif, data_dif))
        distance_self = shortest_path_length(self.crm, s_f[0], self.goal[0])
        # d_t = [s_t[1] for s_t in self.arg_crm.neighbors(s_f)]
        # d_t_ave = sum(d_t)/len(d_t)
        # data_self = abs(self.goal[1]-d_t_ave)
        data_self = abs(self.goal[1]- s_f[1])
        #print('s_f: %s, distance_self: %f, data_self: %f' %(str(s_f), distance_self, data_self))
        #print('sum_distance_dif: %f, sum_data_dif: %f' %(sum_distance_dif, sum_data_dif))
        parameter = np.array([distance_self-sum_distance_dif, data_self-sum_data_dif])
        #print('parameter: (%f, %f)' %(parameter[0],parameter[1]))
        return parameter
        
    def compute_score(self, sa_f, mode='model'):
        parameter = self.compute_parameter(sa_f, mode)
        score = np.dot(self.theta, parameter)
        #print('sa_f:%s, parameter: %s, score: %f' %(str(sa_f),str(parameter), score))
        return score
        
    def policy_approx(self, s_f, mode='model'):
        boltzmann = dict()
        score = dict()
        a_f_list = list(self.crm.neighbors(s_f[0]))
        #print('s_f: %s, a_f_list: %s' %(str(s_f), str(a_f_list)))
        total_exp_score = 0
        for a_f in a_f_list:
            sa_f = (s_f, a_f)
            sa_score = self.compute_score(sa_f, mode)
            score[a_f] = sa_score
            #print('sa_f:%s, exp_score: %f' %(str(sa_f), exp_score))
        max_af_score = max(score.values())
        MAX = 20
        if max_af_score > MAX:
            for af, v in score.items():
                score[af] = score[af]*MAX/max_af_score
        for af,v in score.items():
            exp_score = exp(v)
            boltzmann[af] = exp_score
            total_exp_score += exp_score        
        policy = dict()
        for a_f in boltzmann:
            policy[a_f] = boltzmann[a_f]/total_exp_score
        #print(list(policy.values()))
        return policy

    def act_by_policy(self, policy):
        act_list = list(policy.keys())
        uni_prob_list = [policy[a] for a in act_list]
        prob_list = np.cumsum(uni_prob_list)
        prob = random.random()
        j_list = [j for j,p in enumerate(prob_list) if p>prob]
        if j_list:
            l = j_list[0]
        else:
            l = 0
        #l = [j for j,p in enumerate(prob_list) if p>prob][0]
        return act_list[l]

    def varphi(self, sa_f, mode='model'):
        s_f, a_f = sa_f
        varphi = np.array([0, 0])
        parameter_f = self.compute_parameter(sa_f)
        policy = self.policy_approx(s_f, mode)
        for a_p in policy:
            sa_p = (s_f, a_p)
            parameter_p = self.compute_parameter(sa_p)
            parameter_f -= policy[a_p]*parameter_p
        return parameter_f
        
    def critic_update(self, sa_f, new_sa_f, gamma, mode='model'):
        s_f, a_f = sa_f
        varphi = self.varphi(sa_f, mode)
        new_varphi = self.varphi(new_sa_f, mode)
        new_z = self.clambda*self.z + varphi
        new_b = self.b + gamma*(self.cost(sa_f)*self.z-self.b)
        varphi_dif = new_varphi - varphi
        # print 'self.A,self.z,varphi_dif', self.A, self.z, varphi_dif
        new_A = self.A + gamma*(np.dot(self.z[:,None],varphi_dif[None,:])-self.A)
        new_r = -np.dot(np.linalg.pinv(self.A),self.b)
        self.z = new_z
        self.b = new_b
        self.A = new_A
        self.r = new_r

    def compute_tau(self, r):
        r_norm = np.linalg.norm(r)+0.0001
        return min([self.D/r_norm, 1])
    
    def actor_update(self, new_sa_f, beta, mode='model'):
        new_varphi = self.varphi(new_sa_f, mode)
        d_theta = beta*self.compute_tau(self.r)*np.dot(self.r,new_varphi)*new_varphi
        new_theta = self.theta - d_theta
        self.theta = new_theta
        self.d_theta = d_theta


    def one_episode_learn(self, gamma, beta, mode = 'experiment'):
        print '---one episode starts---'        
        start = time.time()
        print 'initial theta %s' %str(self.theta)
        traj_log = []
        cost_log = []
        theta_log = []
        d_theta_log = []
        d_theta = np.array([0.0, 0.0])
        total_cost = 0
        l = 0
        #---
        s_f = self.init
        policy = self.policy_approx(s_f, mode)
        a_f = self.act_by_policy(policy)
        sa_f = (s_f, a_f)
        while (sa_f[0] != self.goal):
            traj_log.append(sa_f[0])
            theta_log.append((l, (self.theta[0], self.theta[1])))
            d_theta_log.append((l, (self.d_theta[0], self.d_theta[1])))
            s_t, actual_rate = self.successor_deter(sa_f, mode)
            #print('s_t: %s' %str(s_t))
            if mode == 'experiment':
                self.update_arg_crm(sa_f, s_t)
            policy = self.policy_approx(s_t, mode)
            #print('--------------------')
            #print('Policy: %s' %str(policy))
            a_t = self.act_by_policy(policy)
            #print('Chosen action %s' %str(a_t))
            sa_t = (s_t, a_t)
            total_cost += self.cost(sa_t)
            # critic update
            self.critic_update(sa_f, sa_t, gamma, mode)
            # actor update
            self.actor_update(sa_t, beta, mode)
            sa_f = tuple(sa_t)
            l += 1
        print 'one episode done in %d steps, with total cost %d' %(l, total_cost)
        print 'Final theta', self.theta
        cost_log = (l, total_cost)
        episode_learn_log = (traj_log, cost_log, theta_log, d_theta_log)
        return episode_learn_log

    def complete_learn(self, num_episodes, mode="experiment"):
        start = time.time()
        print 'initial theta %s' %str(self.theta)
        gamma_seq = [self.gamma/(k+1) for k in range(num_episodes)]
        beta_seq = [self.beta/((k+1)**2) for k in range(num_episodes)]
        print '-----Actor-critic starts for %d episodes-----' %num_episodes
        print 'initial theta %s' %str(self.theta)
        all_episode_learn_log = []
        k = 0
        while k < num_episodes-1:
            # one episode learning
            # note self.theta updated automatically
            episode_learn_log = self.one_episode_learn(gamma_seq[k], beta_seq[k], mode)
            k += 1
            all_episode_learn_log.append(episode_learn_log)
        total_l = sum([log[1][0] for log in all_episode_learn_log])
        total_c = sum([log[1][1] for log in all_episode_learn_log])
        if k > 0:
            print 'In total %d episodes done in average %.2f steps, with total cost %.2f' %(k, total_l/k, total_c/k)
            print 'Final theta: %s' %str(self.theta)
            print '---- Total time %.2f---' %(time.time()-start)
        else:
            print 'zero repetition done'
            episode_learn_log = ([],[],[[0,list(self.theta)]],[[0.0,0.0]])
            all_episode_learn_log.append(episode_learn_log)
        # pickle.dump(learn_log, open('data/episodes_actor_critic_learn.p', "wb"))
        # print 'data/episodes_actor_critic_learn.p saved'
        return all_episode_learn_log

    
    def MC_run_dijkstra(self, num_runs, max_steps = 1000, deter_plan_file=None,option='static'):
        if deter_plan_file:
            deter_plan = pickle.load(open(deter_plan_file, 'rb'))
        else:
            deter_plan = self.shortest_path
        score = []
        for k in range(num_runs):
            steps = 0
            tot_data = 0
            for s in deter_plan:
                exp_rate = self.crm.node[s[0]]['rate']
                actual_rate = self.uncertain_rate(exp_rate)
                tot_data += actual_rate*self.sample_time
                steps += 1
                if option == 'adaptive':
                    while tot_data < s[1]:
                        exp_rate = self.crm.node[s[0]]['rate']
                        actual_rate = self.uncertain_rate(exp_rate)
                        tot_data += actual_rate*self.sample_time
                        steps += 1                        
            score.append((steps, tot_data))
        print '----%d MC simulation done under adaptive Dijkstra----' %num_runs
        path_leng = [s[0] for s in score]
        tot_data = [s[1] for s in score]
        ave_steps = sum(path_leng)/num_runs
        ave_data = sum(tot_data)/num_runs
        print '[min, ave, max] path length: [%.2f, %.2f, %.2f]; and [min, ave, max] data upload [%.2f, %.2f, %.2f]' %(min(path_leng), ave_steps, max(path_leng), min(tot_data), ave_data, max(tot_data))
        # pickle.dump(score, open('data/dijkstra_%s_score.p' %option, "wb"))
        # print 'data/dijkstra_%s_score.p saved' %option
        return score, (min(path_leng), ave_steps, max(path_leng), min(tot_data), ave_data, max(tot_data))


    def MC_run_learn(self, num_runs, max_steps = 1000, option='actor_critic'):
        # ac_learn_file='data/episodes_actor_critic_learn.p'
        # (cost_log, theta_log, d_theta_log) = pickle.load(open(ac_learn_file, 'rb'))
        # theta = theta_log[-1][1]
        score = []
        for k in range(num_runs):
            s_f = self.init
            policy = self.policy_approx(s_f, 'model')
            a_f = self.act_by_policy(policy)
            sa_f = (s_f, a_f)
            steps = 0
            tot_data = 0
            i = 0
            while (sa_f[0] != self.goal) and (i<= max_steps):
                (s_f, a_f) = sa_f
                s_t, actual_rate = self.successor_deter(sa_f, 'experiment')
                tot_data += actual_rate*self.sample_time
                steps += 1
                policy = self.policy_approx(s_t, 'model')
                a_t = self.act_by_policy(policy)
                sa_f = (s_t, a_t)
                i += 1
            score.append((steps, tot_data))
        print '----%d MC simulation done under %s----' %(num_runs, option)
        path_leng = [s[0] for s in score]
        tot_data = [s[1] for s in score]
        ave_steps = sum(path_leng)/num_runs
        ave_data = sum(tot_data)/num_runs
        print '[min, ave, max] path length: [%.2f, %.2f, %.2f]; and [min, ave, max] data upload [%.2f, %.2f, %.2f]' %(min(path_leng), ave_steps, max(path_leng), min(tot_data), ave_data, max(tot_data))
        # pickle.dump(score, open(file_name, "wb"))
        # print '%s saved' %file_name
        return score, (min(path_leng), ave_steps, max(path_leng), min(tot_data), ave_data, max(tot_data))

    def accumulate(self, L):
        total = 0
        ac_L = []
        for l in L:
            total += l
            ac_L.append(total)
        return ac_L

        

        
