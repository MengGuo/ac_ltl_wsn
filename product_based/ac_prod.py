from math import ceil, floor, exp, sqrt
import numpy as np
import random

import time

def dist_2D(p1, p2):
    return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


class ac_learn(object):
    def __init__(self, mdp, clambda=0.98, gamma=1, beta=1, D=1, theta0=None):
        self.mdp = mdp
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
        print '---- actor_critic learner for mdp: %d nodes and %d edges----' %(len(self.mdp.nodes()), len(self.mdp.edges()))

    def set_init_goal(self, init, goal):
        # single initial state
        # set of goal states
        self.init = init
        self.goal = goal

    def set_mec(self, mec):
        self.mec_set = mec[0]
        self.mec_act = mec[2]

    def set_theta(self, theta):
        self.theta = np.array(theta)
        
    def cost(self, sa_f):
        s_f, a_f = sa_f
        if s_f in self.goal:
            return 0
        else:
            return 1

    def one_episode_learn(self, gamma, beta):
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
        policy = self.policy_approx(s_f)
        a_f = self.act_by_policy(policy)
        sa_f = (s_f, a_f)
        while (sa_f[0] not in self.goal):
            traj_log.append(sa_f[0])
            theta_log.append((l, (self.theta[0], self.theta[1])))
            d_theta_log.append((l, (self.d_theta[0], self.d_theta[1])))
            s_t = self.successor_deter(sa_f)
            #print('s_t: %s' %str(s_t))
            policy = self.policy_approx(s_t)
            #print('--------------------')
            #print('Policy: %s' %str(policy))
            a_t = self.act_by_policy(policy)
            #print('Chosen action %s' %str(a_t))
            sa_t = (s_t, a_t)
            total_cost += self.cost(sa_t)
            # critic update
            self.critic_update(sa_f, sa_t, gamma)
            # actor update
            self.actor_update(sa_t, beta)
            sa_f = tuple(sa_t)
            l += 1
        print 'one episode done in %d steps, with total cost %d, within time %s' %(l, total_cost, str(time.time()-start))
        print 'Final theta', self.theta
        cost_log = (l, total_cost)
        episode_learn_log = (traj_log, cost_log, theta_log, d_theta_log)
        return episode_learn_log        

    def policy_approx(self, s_f):
        boltzmann = dict()
        score = dict()
        comp_a_f_list = list(self.mdp.node[s_f]['act'].copy())
        #--- mec conditions        
        a_f_list = [a for a in comp_a_f_list if a in self.mec_act[s_f]]
        #print('s_f: %s, a_f_list: %s' %(str(s_f), str(a_f_list)))
        total_exp_score = 0
        for a_f in a_f_list:
            sa_f = (s_f, a_f)
            sa_score = self.compute_score(sa_f)
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

    def compute_score(self, sa_f):
        parameter = self.compute_parameter(sa_f)
        score = np.dot(self.theta, parameter)
        #print('sa_f:%s, parameter: %s, score: %f' %(str(sa_f),str(parameter), score))
        return score

    def compute_parameter(self, sa_f):
        #print('**********')
        #print('compute_paramter: sa_f, %s' %str(sa_f))
        s_f, a_f = sa_f
        s_ts, uni_prob_ts = self.successor_prob(sa_f)
        #print('s_ts, %s, uni_prob_ts: %s' %(str(s_ts), str(uni_prob_ts)))
        sum_distance_dif = 0
        sum_data_dif = 0
        for k, s_t in enumerate(s_ts):
            distance_dif = min([dist_2D(s_t[0][0], goal[0][0]) for goal in self.goal])
            sum_distance_dif += uni_prob_ts[k]*distance_dif
            data_dif = min([abs(goal[0][1]- s_t[0][1]) for goal in self.goal])
            sum_data_dif += uni_prob_ts[k]*data_dif
            #print('s_t: %s, distance_self: %f, data_self: %f' %(str(s_t), distance_dif, data_dif))
        distance_self = min([dist_2D(s_f[0][0], goal[0][0]) for goal in self.goal])
        data_self = min([abs(goal[0][1]- s_f[0][1]) for goal in self.goal])
        parameter = np.array([distance_self-sum_distance_dif, data_self-sum_data_dif])
        #print('parameter: (%f, %f)' %(parameter[0],parameter[1]))
        return parameter


    def successor_prob(self, sa_f):
        s_f, a_f = sa_f
        comp_s_ts = list(self.mdp.neighbors(s_f))
        s_ts = []
        uni_prob_ts = []        
        for s_t in comp_s_ts:
            if a_f in self.mdp.edge[s_f][s_t]['prop']:
                s_ts.append(s_t)
                uni_prob_ts.append(self.mdp.edge[s_f][s_t]['prop'][a_f][0])
        return s_ts, uni_prob_ts    

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

    def successor_deter(self, sa_f):
        s_f, a_f = sa_f
        s_ts, uni_prob_ts = self.successor_prob(sa_f)
        prob_list = np.cumsum(uni_prob_ts)
        prob = random.random()
        j_list = [j for j,p in enumerate(prob_list) if p>prob]
        if j_list:
            l = j_list[0]
        else:
            l = 0
        #l = [j for j,p in enumerate(prob_list) if p>prob][0]
        return s_ts[l]
        
    def critic_update(self, sa_f, new_sa_f, gamma):
        s_f, a_f = sa_f
        varphi = self.varphi(sa_f)
        new_varphi = self.varphi(new_sa_f)
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
    
    def actor_update(self, new_sa_f, beta):
        new_varphi = self.varphi(new_sa_f)
        d_theta = beta*self.compute_tau(self.r)*np.dot(self.r,new_varphi)*new_varphi
        new_theta = self.theta - d_theta
        self.theta = new_theta
        self.d_theta = d_theta

    def varphi(self, sa_f):
        s_f, a_f = sa_f
        varphi = np.array([0, 0])
        parameter_f = self.compute_parameter(sa_f)
        policy = self.policy_approx(s_f)
        for a_p in policy:
            sa_p = (s_f, a_p)
            parameter_p = self.compute_parameter(sa_p)
            parameter_f -= policy[a_p]*parameter_p
        return parameter_f        

