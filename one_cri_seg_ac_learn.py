#!/usr/bin/env python

from ac import actor_critic
from crm import build_crm, shortest_route_simple, set_init_goal

from vis import draw_three_paths

import pickle

#----------------------------------------
# load roadmap with wsn rate info, as combined road map (crm)
crm = build_crm()
#----------------------------------------

#----------------------------------------
# set up actor_critic learner
data_bound = 80    # maximal buffer size
quant_size = 1   # buffer quantization size
uncertainty_prob = [0.7, 1.1]         # uncertainty in the wsn rate
Ts = 1             # sampling time
clambda = 0.98      # a_c algorithm parameters
gamma = 1.0
beta = 1.0
D = 0.5
actor_critic_learner = actor_critic(crm, data_bound, quant_size, Ts, uncertainty_prob, clambda, gamma, beta, D)
#----------------------------------------

#----------------------------------------
# initial training of expected model, optional
train_times = 20
for train_time in range(train_times):
    actor_critic_learner.train_arg_crm()
print 'Initial training of system model %d times done!' %train_times
#----------------------------------------

#----------------------------------------
#critical_segments
cri_seg = ((844, 138),(748, 427))
repeat_times = 50
static_learn_episodes = 0
# initialize theta
theta0 = [2.0, 1.0]
#----------------------------------------

#----------------------------------------
# online dynamic actor-critic with LSTD
gamma_seq = [gamma/(k+1) for k in range(repeat_times)]
beta_seq = [beta/((k+1)**2) for k in range(repeat_times)]

#----------------------------
init = (cri_seg[0], data_bound)
goal = (cri_seg[1], 0)
print 'init: %s; goal: %s' %(str(init), str(goal))
init_p, goal_p = set_init_goal(crm, init[0], goal[0])
#----------
# static shortest path in crm
direct_shortest_path = shortest_route_simple(crm, init_p, goal_p)
print 'direct_shortest_path length %d' %len(direct_shortest_path)
#----------
# static shortest path in arg_crm
new_init = (init_p, data_bound)
new_goal = (goal_p, 0)
actor_critic_learner.set_init_goal(new_init, new_goal)
actor_critic_learner.set_theta(theta0)
arg_shortest_path = actor_critic_learner.static_dijkstra()
print 'arg_shortest_path length %d' %len(arg_shortest_path)
#----------------------------

all_learn_log = []
k = 0
while k < repeat_times-1:
    print '--------------k=%d--------------' %k
    #--------------------
    print 'Indirect learning for %d episodes' %static_learn_episodes
    indirect_learn_log = actor_critic_learner.complete_learn(static_learn_episodes, mode ='model')
    #--------------------
    # start moving
    print 'Direct learning for 1 episode.'
    print 'Robot start moving!'
    direct_learn_log = actor_critic_learner.one_episode_learn(gamma_seq[k], beta_seq[k], mode='experiment')        
    #--------------------
    k += 1
    all_learn_log.append([indirect_learn_log, direct_learn_log])
    
pickle.dump(all_learn_log, open('data/one_cri_seg_all_learn_log.p', "wb"))
print 'data/one_cri_seg_all_learn_log.p saved!'


#----------------------------
draw_three_paths(crm, all_learn_log, direct_shortest_path, arg_shortest_path, file_name = './data/one_cri_seg_three_paths.pdf')
