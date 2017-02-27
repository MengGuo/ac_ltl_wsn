#!/usr/bin/env python

from networkx.classes.digraph import DiGraph

from networkx import shortest_path, NetworkXNoPath

import pickle

from math import sqrt

from gurobipy import *


def dist_2D(p1, p2):
    return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


def intp_rate(f_node_rate, t_node_rate):
    # interpolation method
    gamma = 0.3
    intp_rate = gamma*f_node_rate + (1-gamma)*t_node_rate
    return intp_rate


def build_crm(roadmap_dir = './build_roadmap/roadmap_2D.p', wsn_rate_dir = './wsn_routing/rate_map.p', ws_img_dir = './build_roadmap/ws_img.p'):
    '''
    build combined roadmap, crm
    via load roadmap and merge in wsn_rate info
    '''
    roadmap_edges = pickle.load(open(roadmap_dir, 'rb'))    
    wsn_rate, wifis_loc, sink_loc = pickle.load(open(wsn_rate_dir, 'rb'))
    img = pickle.load(open(ws_img_dir, 'rb'))
    crm = DiGraph(name='combined_model', ws_img=img, wifis=wifis_loc, sink=sink_loc)
    for e in roadmap_edges:
        (f_node, t_node) = e
        near_f_node = min(wsn_rate.keys(), key=lambda p: dist_2D(p, f_node))
        f_node_rate = wsn_rate[near_f_node]
        crm.add_node(f_node, rate=f_node_rate)
        near_t_node = min(wsn_rate.keys(), key=lambda p: dist_2D(p, t_node))
        t_node_rate = wsn_rate[near_t_node]
        crm.add_node(t_node, rate=t_node_rate)
        # interpolation
        dist_e = dist_2D(f_node, t_node)
        f_t_rate = intp_rate(f_node_rate, t_node_rate)    
        crm.add_edge(f_node, t_node, rate=f_t_rate, dist=dist_e)
        t_f_rate = intp_rate(t_node_rate, f_node_rate)    
        crm.add_edge(t_node, f_node, rate=t_f_rate, dist=dist_e)
    print '---crm constructed from %s and %s, %d nodes, %d edges---' %(roadmap_dir, wsn_rate_dir, len(crm.nodes()), len(crm.edges()))
    return crm


def set_init_goal(crm, init_point, goal_point):
    if not init_point in crm.nodes():
        near_init_point = min(crm.nodes(), key=lambda p:dist_2D(p, init_point))
        print 'init_point %s not in roadmap, use closest point %s instead' %(str(init_point), str(near_init_point))
        init_point = near_init_point
    if not goal_point in crm.nodes():
        near_goal_point = min(crm.nodes(), key=lambda p: dist_2D(p, goal_point))
        print 'goal_point %s not in roadmap, use closest point %s instead' %(str(goal_point), str(near_goal_point))
        goal_point = near_goal_point
    crm.graph['init'] = init_point
    crm.graph['goal'] = goal_point
    return init_point, goal_point


def shortest_route_simple(crm, init_point, goal_point):
    print '---static 2D dijkstra starts---'
    if not init_point in crm.nodes():
        near_init_point = min(crm.nodes(), key=lambda p: dist_2D(p, init_point))
        print 'init_point %s not in roadmap, use closest point %s instead' %(str(init_point), str(near_init_point))
        init_point = near_init_point
    if not goal_point in crm.nodes():
        near_goal_point = min(crm.nodes(), key=lambda p: dist_2D(p, goal_point))
        print 'goal_point %s not in roadmap, use closest point %s instead' %(str(goal_point), str(near_goal_point))
        goal_point = near_goal_point
    try:
        route = shortest_path(crm, source=init_point, target=goal_point)
        print 'Simple shortest route found! total length: %d' %len(route)
    except NetworkXNoPath:
        print 'No path found from init to goal'
        print 'Try again or consider more samples'
        route = []
    return route


def constrainted_shortest_route_MILP(crm, gamma, Ts):
    # from gurobipy import *, first
    start = time.time()
    print('-----')
    print('Gurobi starts now')
    print('-----')
    try:
        Y = dict()
        X = dict()
        model = Model('constrainted_SP')
        # create variables
        for (f_node, t_node) in crm.edges_iter():
            Y[(f_node,t_node)] = model.addVar(vtype=GRB.BINARY,name='Y[(%s, %s)]' %(f_node, t_node))
            X[f_node] = model.addVar(vtype=GRB.INTEGER,lb=0,name='X[%s]' %str(f_node))
        model.update()
        print('Variables added')
        # set objective
        obj = 0
        for (f_node, t_node) in crm.edges_iter():
            obj += Y[(f_node,t_node)]
        model.setObjective(obj, GRB.MINIMIZE)
        print('Objective function set')
        # add constraints
        #------------------------------
        total_gamma = 0
        N = len(crm.nodes())
        for (f_node, t_node) in crm.edges_iter():
            total_gamma += crm.node[f_node]['speed']*Ts*Y[(f_node,t_node)]
            model.addConstr(X[f_node]-X[t_node] + N*Y[(f_node,t_node)] <= N-1, 'cycle_%s_%s' %(str(f_node), str(t_node)))
        model.addConstr(total_gamma >= gamma, 'total_gamma')
        #----------------------------------------
        init_node = crm.graph['init']        
        goal_node = crm.graph['goal']
        for f_node in crm.nodes_iter():
            degree_in = 0
            degree_out = 0
            for t_node in crm.successors_iter(f_node):
                degree_out += Y[(f_node, t_node)]
            for ff_node in crm.predecessors_iter(f_node):
                degree_in += Y[(ff_node, f_node)]
            if ((f_node !=init_node) and (f_node !=goal_node)):
                model.addConstr(degree_in == degree_out, 'balance')
            if f_node == init_node:
                model.addConstr(degree_in == 0, 'init_in')
                model.addConstr(degree_out == 1, 'init_out')
            if f_node == goal_node:
                model.addConstr(degree_in == 1, 'goal_in')
                model.addConstr(degree_out == 0, 'goal_out')
        #------------------------------
        #------------------------------
        print('--optimization starts--')
        model.optimize()
        # print '--variables value--'
        # for v in model.getVars():
        #     print v.varName, v.x
        print('obj:', model.objVal)
        #------------------------------
        total_gamma = 0
        route = []
        node = init_node
        while node != goal_node:
            route.append(node)
            for t_node in crm.successors_iter(node):
                if Y[(node, t_node)].X >= 0.9:
                    node = tuple(t_node)
                    # print 'node speed', crm.node[node]['speed']
                    total_gamma += crm.node[node]['speed']*Ts
                    break
        route.append(goal_node)
        crm.route = route
        # print route
        #------------------------------
        print('--Constrained shortest path computed---')
        print('total length:', len(route))
        print('total data uploaded: %s Mb, and required %s Mb') %(total_gamma, gamma)
        pickle.dump(route, open('final_route_%s.p' %str(gamma), "wb"))
        print('Best route saved to find_route_%s.p' %str(gamma))
        print('----Total time %.2f----' %time.time()-start)
        return route, total_gamma
    except GurobiError:
        print("Gurobi Error reported")
        return None, 0


def constrainted_shortest_route_MILP(rrt, gamma, Ts):
    start = time.time()
    print('-----')
    print('Gurobi starts now')
    print('-----')
    try:
        Y = dict()
        X = dict()
        model = Model('constrainted_SP')
        # create variables
        for (f_node, t_node) in rrt.edges_iter():
            Y[(f_node,t_node)] = model.addVar(vtype=GRB.BINARY,name='Y[(%s, %s)]' %(f_node, t_node))
            X[f_node] = model.addVar(vtype=GRB.INTEGER,lb=0,name='X[%s]' %str(f_node))
        model.update()
        print('Variables added')
        # set objective
        obj = 0
        for (f_node, t_node) in rrt.edges_iter():
            obj += Y[(f_node,t_node)]
        model.setObjective(obj, GRB.MINIMIZE)
        print('Objective function set')
        # add constraints
        #------------------------------
        total_gamma = 0
        N = len(rrt.nodes())
        for (f_node, t_node) in rrt.edges_iter():
            total_gamma += rrt.node[f_node]['speed']*Ts*Y[(f_node,t_node)]
            model.addConstr(X[f_node]-X[t_node] + N*Y[(f_node,t_node)] <= N-1, 'cycle_%s_%s' %(str(f_node), str(t_node)))
            # model.addConstr(Y[(f_node,t_node)] + Y[(t_node,f_node)] <= 1, 'equal_%s_%s' %(str(f_node), str(t_node)))
        model.addConstr(total_gamma >= gamma, 'total_gamma')
        #----------------------------------------
        start_node = rrt.graph['start']        
        goal_node = rrt.graph['goal']
        for f_node in rrt.nodes_iter():
            degree_in = 0
            degree_out = 0
            for t_node in rrt.successors_iter(f_node):
                degree_out += Y[(f_node, t_node)]
            for ff_node in rrt.predecessors_iter(f_node):
                degree_in += Y[(ff_node, f_node)]
            if ((f_node !=start_node) and (f_node !=goal_node)):
                model.addConstr(degree_in == degree_out, 'balance')
            if f_node == start_node:
                model.addConstr(degree_in == 0, 'start_in')
                model.addConstr(degree_out == 1, 'start_out')
            if f_node == goal_node:
                model.addConstr(degree_in == 1, 'goal_in')
                model.addConstr(degree_out == 0, 'goal_out')
        #------------------------------
        #------------------------------
        print('--optimization starts--')
        model.optimize()
        # print '--variables value--'
        # for v in model.getVars():
        #     print v.varName, v.x
        print('obj:', model.objVal)
        #------------------------------
        total_gamma = 0
        route = []
        node = start_node
        while node != goal_node:
            route.append(node)
            for t_node in rrt.successors_iter(node):
                if Y[(node, t_node)].X >= 0.9:
                    node = tuple(t_node)
                    # print 'node speed', rrt.node[node]['speed']
                    total_gamma += rrt.node[node]['speed']*Ts
                    break
        route.append(goal_node)
        rrt.route = route
        # print route
        #-----------------------------
        # for (f_node, t_node) in rrt.edges_iter():
        #     if Y[(f_node, t_node)].X >= 0.9:
        #         print 'edge', (f_node, t_node)
        #         print 'speed', rrt.node[f_node]['speed']
        #------------------------------
        print('--Constrained shortest path computed---')
        print('total length:', len(route))
        print('total data uploaded: %s Mb, and required %s Mb') %(total_gamma, gamma)
        pickle.dump(route, open('final_route_%s.p' %str(gamma), "wb"))
        print('Best route saved to find_route_%s.p' %str(gamma))
        print('----Total time %.2f----' %time.time()-start)
        return route, total_gamma
    except GurobiError:
        print("Gurobi Error reported")
        return None, 0    
