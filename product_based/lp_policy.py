from MDP_TG.mdp import Motion_MDP
from MDP_TG.dra import Dra, Product_Dra
from MDP_TG.lp import syn_full_plan

import pickle
import time

start = time.time()


node_dict, edge_dict, U, initial_node, initial_label = pickle.load(open('./mdp.p','rb'))
#----
motion_mdp = Motion_MDP(node_dict, edge_dict, U, initial_node, initial_label)

print 'motion_mdp done by time %s' %str(time.time()-start)
#----
# task_formula = "& G F & r1 g1 & G F & r2 g2 G F & r4 g3"
task_formula = "& G F & r1 g1 & G F & r2 g2 & G F & r3 F & r4 g3 G F & r2 g1"
dra = Dra(task_formula)

print 'dra done by time %s' %str(time.time()-start)

#----
prod_dra = Product_Dra(motion_mdp, dra)
prod_dra.compute_S_f()
#prod_dra.dotify()

print 'prod_dra done by time %s' %str(time.time()-start)

allowed_risk = 0.0
best_all_plan = syn_full_plan(prod_dra, allowed_risk)

# #----------------------------------------
# print "----------------------------------------"
# print "||||||||Simulation start||||||||||||||||"
# print "----------------------------------------"
# total_T = 20
# state_seq = [initial_node,]
# label_seq = [initial_label,]
# N = 5
# n = 0
# print "Try %s simulations of length %s" %(str(N), str(total_T))

# XX = []
# LL = []
# UU = []
# MM = []
# PP = []

# while (n < N):
#     print '=======simulation %s starts=======' %str(n)
#     X, L, U, M, PX = prod_dra.execution(best_all_plan, total_T, state_seq, label_seq)
#     # print 'Product State trajectory: %s' %str(PX)
#     # print 'State trajectory: %s' %str(X)
#     # print 'Label trajectory: %s' %str(L)
#     # print 'Control Actions: %s' %str(U)
#     # print 'Marker sequence: %s' %str(M)
#     print '=======simulation %s ends=======' %str(n)
#     XX.append(X)
#     LL.append(L)
#     UU.append(U)
#     MM.append(M)
#     PP.append(PX)
#     n += 1

#visualize_run(XX, LL, UU, MM, 'surv_result')    

print 'lp_policy done by time %s' %str(time.time()-start)
