#!/usr/bin/env python

from ac import actor_critic
from crm import build_crm, shortest_route_simple, set_init_goal
from ltl import load_plan, find_cri_segs

from vis import draw_score

import pickle

#----------------------------------------
# load roadmap with wsn rate info, as combined road map (crm)
crm = build_crm()
#----------------------------------------

#----------------------------------------
# load motion and action plan
ma_plan, ma_path, act = load_plan()
data_bound = 80    # maximal buffer size
# cri_segs = [((pi,ai), (pj,aj))...]
# cri_segs_index = [(i,j)...]
cri_segs, cri_segs_index = find_cri_segs(ma_plan, ma_path, act, data_bound)
repeat_times = 10


#----------------------------------------
# set up actor_critic learner
data_bound = 80    # maximal buffer size
quant_size = 1   # buffer quantization size
uncertainty_prob = [0.5, 1.1]    # uncertainty in the wsn rate
Ts = 1             # sampling time
clambda = 0.9      # a_c algorithm parameters
gamma = 1.0
beta = 1.0
D = 0.5
actor_critic_learner = actor_critic(crm, data_bound, quant_size, Ts, uncertainty_prob, clambda, gamma, beta, D)
#----------------------------------------

#----------------------------------------
# load Theta
Theta = pickle.load(open('./data/ltl_ac_theta.p','rb'))
    
#----------------------------------------

#----------------------------------------
# online dynamic actor-critic with LSTD
MC_log = dict()
num_runs = 200
max_steps = 100

for cri_seg in cri_segs:
    init = (cri_seg[0][0], data_bound)
    goal = (cri_seg[1][0], 0)
    print '||||||||ac learning---init: %s; goal: %s|||||||' %(str(init), str(goal))
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
    arg_shortest_path = actor_critic_learner.static_dijkstra()
    print 'arg_shortest_path length %d' %len(arg_shortest_path)
    #----------
    # _score: score (steps,tot_data), (min(path_leng), ave_steps, max(path_leng), min(tot_data), ave_data, max(tot_data))
    dijk_score = actor_critic_learner.MC_run_dijkstra(num_runs, max_steps,  option='adaptive')
    actor_critic_learner.set_theta(Theta[cri_seg])
    ac_score = actor_critic_learner.MC_run_learn(num_runs, max_steps, option='actor_critic')
    MC_log[cri_seg]=[dijk_score, ac_score]

pickle.dump(MC_log, open('./data/ltl_ac_MC_log.p', "wb"))
print './data/ltl_ac_MC_log.p saved!'

#----------------------------
draw_score(MC_log, file_name='./data/ltl_ac_MC_score.pdf')

