from MDP_TG.mdp import Motion_MDP
from MDP_TG.dra import Dra, Product_Dra
from ac_prod import ac_learn

import pickle


node_dict, edge_dict, U, initial_node, initial_label = pickle.load(open('./mdp.p','rb'))
#----
motion_mdp = Motion_MDP(node_dict, edge_dict, U, initial_node, initial_label)

#----
task_formula = "& G F (& r1 g1) & G F (& r2 g2) G F (& r4 g3)"
# task_formula = "& G F (& r1 g1) & G F (& r2 g2) G F & r3 F (& r4 g3)"
dra = Dra(sur_formula1)

#----
prod_dra = Product_Dra(motion_mdp, dra)
prod_dra.compute_S_f()
#prod_dra.dotify()

clambda = 0.98      # a_c algorithm parameters
gamma = 1.0
beta = 1.0
D = 0.5
actor_critic_learner = ac_learn(prod_dra, clambda, gamma, beta, D)


repeat_times = 50
theta0 = [2.0, 1.0]

gamma_seq = [gamma/(k+1) for k in range(repeat_times)]
beta_seq = [beta/((k+1)**2) for k in range(repeat_times)]

# accept = prod_dra.graph['accept']
# init = accept[0][0][0]
# goal = prod_dra.predecessors(init)
mec = prod_dra.Sf[0]
init = list(mec[1])[0]
goal = prod_dra.predecessors(init)
actor_critic_learner.set_init_goal(init, goal)
actor_critic_learner.set_theta(theta0)
actor_critic_learner.set_mec(mec)

all_learn_log = []
k = 0
while k < repeat_times-1:
    print '--------------k=%d--------------' %k
    direct_learn_log = actor_critic_learner.one_episode_learn(gamma_seq[k], beta_seq[k])
    #--------------------
    k += 1
    all_learn_log.append(direct_learn_log)
