#!/usr/bin/env python

from networkx.classes.digraph import DiGraph

from math import ceil, floor, exp

import pickle

from math import sqrt

import numpy as np

import time


def dist_2D(p1, p2):
    return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


def uncertain_rate(exp_rate, uncertainty=[0.7,1.1]):
    # triangular distribution as an example
    actual_rate = np.random.triangular(exp_rate*uncertainty[0], exp_rate, exp_rate*uncertainty[1])
    return actual_rate


def intp_rate(f_node_rate, t_node_rate):
    # interpolation method
    gamma = 0.3
    intp_rate = gamma*f_node_rate + (1-gamma)*t_node_rate
    return intp_rate


def unify_prob(node_dict, Us, edge_dict):
    print 'node numbers: %d' %len(node_dict)
    nu = 0
    for f_node in node_dict.keys():
        print 'node %d'%nu
        nu += 1
        for u in Us[f_node]:
            sum_weight = 0
            edges = [e for e in edge_dict.keys() if e[0]==f_node if e[1]==u]
            for e in edges:
                sum_weight += edge_dict[e][0]
            for e in edges:
                edge_dict[e][0] = edge_dict[e][0]/sum_weight
    return edge_dict


def build_mdp(data_bound=80,
              quant_size=1,
              Ts=1,
              uncertainty=[0.7,1.1],
              initial=[((556,373),0),'r1'],
              roadmap_dir = '../build_roadmap/roadmap_2D.p',
              wsn_rate_dir = '../wsn_routing/rate_map.p',
              ws_img_dir = '../build_roadmap/ws_img.p',
              label_dir='../ltl_plan/state_action.p'):
    '''
    build combined roadmap, mdp
    via load roadmap and merge in wsn_rate info
    '''
    start = time.time()
    roadmap_edges = pickle.load(open(roadmap_dir, 'rb'))    
    wsn_rate, wifis_loc, sink_loc = pickle.load(open(wsn_rate_dir, 'rb'))
    img = pickle.load(open(ws_img_dir, 'rb'))
    state_label, act = pickle.load(open(label_dir, 'rb'))
    data_set = [k*quant_size for k in range(int(floor(data_bound/quant_size))+1)]
    print 'roadmap_edges number %d' %len(roadmap_edges)
    # ----------------------------------------
    # inputs to MDP_TG
    node_dict = dict()
    edge_dict = dict()
    U = set()
    Us = dict()
    initial_node = initial[0]
    initial_label = set([initial[1],])
    # ----------------------------------------
    # add edges
    nu = 0
    for e_bi in roadmap_edges:
        nu += 1
        print 'edge %d' %nu
        (f_wp, t_wp) = e_bi
        e_dir = [(f_wp, t_wp),(t_wp, f_wp)]
        for e in e_dir:
            (f_wp, t_wp) = e
            for f_d in data_set:
                    # f_node
                    near_f_wp = min(wsn_rate.keys(), key=lambda p: dist_2D(p, f_wp))
                    f_wp_rate = wsn_rate[near_f_wp]
                    near_t_wp = min(wsn_rate.keys(), key=lambda p: dist_2D(p, t_wp))
                    t_wp_rate = wsn_rate[near_t_wp]
                    f_node = (f_wp, f_d)
                    if f_node not in node_dict:
                        if f_wp not in state_label.keys():
                            node_dict[f_node] = {frozenset(): 1.0,}
                        else:
                            prop = state_label[f_wp]
                            prob_label = {frozenset([prop,]): 1.0,}
                            node_dict[f_node] = dict(prob_label)
                    # transition by move and transmit
                    # dist_e = dist_2D(f_wp, t_wp)                    
                    f_t_rate = intp_rate(f_wp_rate, t_wp_rate)
                    for i in range(100):
                        actual_f_t_rate = uncertain_rate(f_t_rate, uncertainty)
                        actual_t_d = f_d - actual_f_t_rate*Ts
                        for t_d in data_set:
                            if actual_t_d<=t_d<=actual_t_d+quant_size:
                                t_node = (t_wp, t_d)
                                if t_wp not in state_label.keys():
                                    node_dict[t_node] = {frozenset(): 1.0,}
                                else:
                                    prop = state_label[t_wp]
                                    prob_label = {frozenset([prop,]): 1.0,}
                                    node_dict[t_node] = dict(prob_label)
                                if (f_node, (t_wp,1), t_node) not in edge_dict:
                                    edge_dict[(f_node, (t_wp,1), t_node)] = [1, 1] # frequency for probility, uniform cost 1 (could be replaced by dist_e)
                                    U.add((t_wp,1))
                                    if f_node not in Us:
                                        Us[f_node] = set([(t_wp,1)])
                                    else:
                                        Us[f_node].add((t_wp,1))
                                else:
                                    edge_dict[(f_node, (t_wp,1), t_node)][0] += 1
                    # transition by only move
                    t_node_move = (t_wp, f_d)
                    if (f_node, (t_wp,0), t_node_move) not in edge_dict:
                        edge_dict[(f_node, (t_wp,0), t_node_move)] = [1, 1]
                        U.add((t_wp,0))
                        if f_node not in Us:
                            Us[f_node] = set([(t_wp,0)])
                        else:
                            Us[f_node].add((t_wp,0))
                    # transition by action
                    if f_wp in state_label.keys():
                        prop = state_label[f_wp]
                        for g_act,attri in act.items():
                            if g_act != 'g0':
                                data = attri[0]
                                if f_d + data <= data_bound:
                                    t_d_min = [d for d in data_set if d>=f_d+data][0]
                                    t_node_act = (f_wp, t_d_min, tuple(g_act))
                                    if t_node_act not in node_dict:
                                        prob_label = {frozenset([prop, g_act]): 1.0,}
                                        node_dict[t_node_act] = dict(prob_label)
                                    if (f_node, tuple(g_act), t_node_act) not in edge_dict:
                                        edge_dict[(f_node, tuple(g_act), t_node_act)] = [1, 1]
                                        print 'added action edge', (f_node, tuple(g_act), t_node_act)
                                        U.add(tuple(g_act))
                                        if f_node not in Us:
                                            Us[f_node] = set([tuple(g_act),])
                                        else:
                                            Us[f_node].add(tuple(g_act))
                                    tt_node_act = (f_wp, t_d_min)
                                    if tt_node_act not in node_dict:
                                        prob_label = {frozenset([prop,]): 1.0,}
                                        node_dict[tt_node_act] = dict(prob_label)
                                    if (t_node_act, (f_wp,0), tt_node_act) not in edge_dict:
                                        edge_dict[(t_node_act, (f_wp,0), tt_node_act)] = [1, 1]
                                        U.add((f_wp,0))
                                        if t_node_act not in Us:
                                            Us[t_node_act] = set([(f_wp,0),])
                                        else:
                                            Us[t_node_act].add((f_wp,0))
    # ----------------------------------------
    # unify transition probabilities 
    edge_dict = unify_prob(node_dict, Us, edge_dict)
    print '---mdp constructed from %s and %s, %d nodes, %d edges---' %(roadmap_dir, wsn_rate_dir, len(node_dict), len(edge_dict))
    pickle.dump([node_dict, edge_dict, U, initial_node, initial_label],open('./mdp.p','wb'))
    print '---mdp data files saved at ./mdp.p---'
    print '---mdp data generated by time: %s---' %str(time.time()-start)



if __name__ == "__main__":
    build_mdp()
