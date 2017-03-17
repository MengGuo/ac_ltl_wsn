#!/usr/bin/env python

from ac import actor_critic
from crm import build_crm, shortest_route_simple, set_init_goal
from ltl import load_plan, find_cri_segs

from vis import draw_init_plan, draw_data, draw_theta, make_movie

import pickle

import time

start = time.time()
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
repeat_times = 30
static_learn_episodes = 0


#----------------------------------------
# set up actor_critic learner
data_bound = 80    # maximal buffer size
quant_size = 1   # buffer quantization size
uncertainty_prob = [0.7, 1.1]    # uncertainty in the wsn rate
Ts = 1             # sampling time
clambda = 0.98      # a_c algorithm parameters
gamma = 10.0
beta = 10.0
D = 0.5
actor_critic_learner = actor_critic(crm, data_bound, quant_size, Ts, uncertainty_prob, clambda, gamma, beta, D)
#----------------------------------------


#----------------------------------------
# initial training of expected model, optional
train_times = 1
for train_time in range(train_times):
    actor_critic_learner.train_arg_crm()
print 'Initial training of system model %d times done!' %train_times
#----------------------------------------

#----------------------------------------
Theta = dict()
Dijk_log = dict()
Learn_log = dict()
# initialize theta
for e in cri_segs:
    Theta[e] = [1.0, 0.1]
    Dijk_log[e] = []
    Learn_log[e] = []
    
#----------------------------------------

#----------------------------------------
# online dynamic actor-critic with LSTD
t = 0
Robot_status_log = []
gamma_seq = [gamma/(k+1) for k in range(repeat_times)]
beta_seq = [beta/((k+1)**2) for k in range(repeat_times)]

k = 0
while k < repeat_times:
    i = 0
    # time, pose, action, buffer, Learning_seg, theta
    robot_pose = ma_plan[0]
    robot_buffer = 0
    episode_robot_status_log = []
    episode_robot_status_log.append([t, list(robot_pose), None, [0, data_bound], None, None])
    while i < len(ma_plan):
        print '--------- %d-th repetition, %d-th state in plan --------------' %(k, i)
        todo = ma_plan[i]
        #---------------------
        if isinstance(todo, basestring):
            # action
            data, duration = act[todo]
            # t +=  duration
            t += 1
            robot_buffer += data
            print 'action %s performed at waypoint %s, data increased by %.2f to %.2f' %(todo, robot_pose, data, robot_buffer)
            episode_robot_status_log.append([t, list(robot_pose), todo, [robot_buffer, data_bound], None, None])
        else:
            # motion via normal navigation
            init = list(robot_pose)
            goal = ma_path[i][0]
            print '|||||||navigation---init: %s; goal: %s|||||||' %(str(init), str(goal))
            init_p, goal_p = set_init_goal(crm, init, goal)
            #----------
            # static shortest path in crm
            direct_shortest_path = shortest_route_simple(crm, init_p, goal_p)
            print 'direct_shortest_path for navigation found! length %d' %len(direct_shortest_path)
            for p in direct_shortest_path[1:]:
                t += Ts
                robot_pose = list(p)
                episode_robot_status_log.append([t, list(p), None, [robot_buffer, data_bound], None, None])
            if tuple(init) in ma_plan:
                seg = tuple(((tuple(init),0), (tuple(goal),0)))
                if seg not in Dijk_log:
                    Dijk_log[seg] = []
                Dijk_log[seg].append([t, direct_shortest_path, None])                
            print 'navigation from %s to %s done, via path %s' %(str(init), str(goal), str(direct_shortest_path))
        # ------------------------------            
        if i not in [e[0] for e in cri_segs_index]:
            i += 1
        else:
            # with learning
            #----------------------------
            (i, j) = [e for e in cri_segs_index if e[0]==i][0]
            cri_seg = (ma_path[i], ma_path[j])
            # init = (cri_seg[0][0], data_bound)
            init = (cri_seg[0][0], robot_buffer)
            goal = (cri_seg[1][0], 0)
            print '||||||||ac learning---init: %s; goal: %s|||||||' %(str(init), str(goal))
            init_p, goal_p = set_init_goal(crm, init[0], goal[0])
            #----------
            # static shortest path in crm
            direct_shortest_path = shortest_route_simple(crm, init_p, goal_p)
            print 'direct_shortest_path length %d' %len(direct_shortest_path)
            #----------
            # static shortest path in arg_crm
            new_init = (init_p, robot_buffer)
            new_goal = (goal_p, 0)
            actor_critic_learner.set_init_goal(new_init, new_goal)                
            arg_shortest_path = actor_critic_learner.static_dijkstra()
            print 'arg_shortest_path length %d' %len(arg_shortest_path)
            Dijk_log[cri_seg].append([t, direct_shortest_path, arg_shortest_path])
            #----------------------------
            actor_critic_learner.set_theta(Theta[cri_seg])
            print '|||||||Indirect learning for %d episodes|||||||' %static_learn_episodes
            indirect_learn_log = actor_critic_learner.complete_learn(static_learn_episodes, mode ='model')
            #--------------------
            # start moving
            print '|||||||Direct learning for 1 episode|||||||'
            print 'Robot starts moving from %s to %s via learning!' %(str(new_init), str(new_goal)) 
            direct_learn_log = actor_critic_learner.one_episode_learn(gamma_seq[k], beta_seq[k], mode='experiment')
            #  episode_learn_log = (traj_log, cost_log, theta_log, d_theta_log)
            (traj_log, cost_log, theta_log, d_theta_log) = direct_learn_log
            t_pre = t
            for l,p in enumerate(traj_log[1:]):
                t += Ts
                robot_pose = list(p[0])
                robot_buffer = p[1]
                episode_robot_status_log.append([t, list(robot_pose), None, [robot_buffer, data_bound], cri_seg,list(theta_log[l][1])])
            Learn_log[cri_seg].append([t_pre, t, indirect_learn_log, direct_learn_log])
            if j < i:
                i = len(ma_plan)
            else:
                i = j
            Theta[cri_seg] = list(actor_critic_learner.theta)
        #--------------------
    k += 1
    Robot_status_log.append(list(episode_robot_status_log))

print '========================================'
print 'Simulation done in %.2f' %(time.time()-start)
print 'Learning time: %d' %t
print '========================================'
    
pickle.dump(Robot_status_log, open('./data/ltl_ac_learn_Robot_status_log.p', "wb"))
print './data/ltl_ac_learn_Robot_status_log.p saved!'

pickle.dump(Dijk_log, open('./data/ltl_ac_learn_dijk_log.p', "wb"))
print './data/ltl_ac_learn_dijk_log.p saved!'

pickle.dump(Learn_log, open('./data/ltl_ac_learn_log.p', "wb"))
print './data/ltl_ac_learn_log.p saved!'

print '=============================='
print 'Final theta', Theta
print '=============================='

pickle.dump(Theta, open('./data/ltl_ac_theta.p', "wb"))
print './data/ltl_ac_theta.p saved!'

#----------------------------
draw_init_plan(crm, Dijk_log, file_name = './data/ltl_ac_init_plan.pdf')
# draw_data(Robot_status_log, file_name='./data/ltl_ac_buffer.pdf')
draw_theta(Learn_log, file_name='./data/ltl_ac_theta.pdf')

num_pics = 200
make_movie(crm, num_pics, Robot_status_log, Dijk_log, movie_name='ltl_ac_learn')

