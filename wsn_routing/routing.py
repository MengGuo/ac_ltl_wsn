#!/usr/bin/env python

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import cm

from gurobipy import *
from collections import defaultdict
from wifi import wifi_2D

import networkx

import pickle
import numpy as np


import time

matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True


def dist_2D(p1, p2):
    return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


def compose_wifi_rate(wifis, source, sink, file_name='./wsn_rate.p', step=100):
    print '--------------------'
    wifi = wifis[0]
    ws_img = wifi.ws_img
    total_y = len(ws_img)
    total_x = len(ws_img[0])
    wsn_rate = dict()
    print 'total_x, total_y', total_x, total_y
    #---- max rate for sampled points
    for point_x in range(0, total_x, step):
        for point_y in range(0, total_y, step):
            point = (point_x, point_y)
            loc_rate = find_max_rate(point, source, wifis, sink)
            wsn_rate[point] = loc_rate
    wifis_loc = [w.loc for w in wifis]
    pickle.dump([wsn_rate, wifis_loc, sink.loc], open(file_name, "wb"))
    print '---%s saved---' %file_name
    return wsn_rate

    
def find_max_rate(node, source, wifis, sink):
    print '----------------------------------------'
    print 'To determine max rate at %s' %str(node)
    N_wifis = len(wifis)
    wifi_heat = dict()
    source_heat = dict()
    for i in range(1, N_wifis+1):
        for j in range(1, N_wifis+1):
            if i != j:
                wifi_heat[(i,j)]= wifis[i-1].wifi_snr(wifis[j-1].loc)
            else:
                wifi_heat[(i,j)]= 0              
        wifi_heat[(i,0)] = wifis[i-1].wifi_snr(sink.loc)
    for j in range(1, N_wifis+1):
        source_heat[(0,j)]= wifis[j-1].wifi_snr(node)
    source_heat[(0,0)]= sink.wifi_snr(node)
    print 'source_heat, wifi_heat Constructed'
    print 'source_heat:', source_heat
    print 'wifi_heat:', wifi_heat
    # -------------
    # source 0 (only send), wifis 1-K, coordinator 0 (only receive)
    # formulate LP
    print '---Gurobi starts now---'
    try:
        Y = defaultdict(float)
        model = Model('point_routing')
        # create variables
        for i in range(0, N_wifis+1):
            for j in range(0, N_wifis+1):
                Y[(i,j)] = model.addVar(vtype=GRB.CONTINUOUS,lb=0,ub=1,name='y[(%s, %s)]' %(i, j))
        model.update()
        print 'Variables added'
        # set objective
        obj = 0
        for j in range(0, N_wifis+1):
            if j>0:
                #obj += Y[(0,j)]*source.rate*wifi_heat[(j,0)]
                obj += Y[(0,j)]*source.rate*source_heat[(0,j)]
            else:
                obj += Y[(0,j)]*source.rate*source_heat[(0,0)]
        model.setObjective(obj, GRB.MAXIMIZE)
        print 'Objective function set'
        # add constraints
        #------------------------------
        Y_out = 0
        #----------------------------------------
        for j in range(0, N_wifis+1):
            Y_out += Y[(0,j)]
        model.addConstr(Y_out <= 1, 'Y_out_node')
        #----------------------------------------        
        for i in range(1, N_wifis+1):
            Y_out = 0
            total_wifi_out = 0
            total_wifi_in = 0
            for j in range(0, N_wifis+1):
                # print 'j, i', j, i
                Y_out += Y[(i,j)]
                total_wifi_out += wifis[i-1].rate*Y[(i,j)]*wifi_heat[(i,j)]
                if j > 0:
                    total_wifi_in += wifis[j-1].rate*Y[(j,i)]*wifi_heat[(j,i)]
            #total_wifi_in += source.rate*Y[(0,j)]*source_heat[(0,j)]
            total_wifi_in += source.rate*Y[(0,i)]*source_heat[(0,i)]
            model.addConstr(Y_out <= 1, 'Y_out_%s' %str(i))
            model.addConstr(total_wifi_out <= total_wifi_in, 'Y_balance1_%s' %str(i))
            model.addConstr(total_wifi_out >= total_wifi_in, 'Y_balance2_%s' %str(i))
        #----------------------------------------
        total_wifi_in = 0
        for i in range(1, N_wifis+1):
            total_wifi_in += Y[(i,0)]*wifis[i-1].rate*wifi_heat[(i,0)]
        total_wifi_in += Y[(0,0)]*source.rate*source_heat[(0,0)]
        model.addConstr(sink.rate >= total_wifi_in, 'Y_balance_sink')
        #------------------------------
        print '---optimization starts---'
        model.optimize()
        # print '---variables value---'
        for v in model.getVars():
            print v.varName, v.x
        print 'obj:', model.objVal 
        #------------------------------
        print '---maximal upload rate at sampled points computed---'
        return model.objVal
    except GurobiError:
        print "Gurobi Error reported"
        return 0            


def draw_wsn_rate(wsn_rate_file_dir, wifis, sink, file_name='wsn_rate.pdf'):
    wsn_rate, wifis_loc, sink.loc = pickle.load(open(wsn_rate_file_dir,'rb'))    
    # ---------- process data
    X = set()
    Y = set()
    intensity = []
    for k in wsn_rate.iterkeys():
        X.add(k[0])
        Y.add(k[1])
    Y = list(Y)
    X = list(X)
    Y.sort()
    X.sort()
    for y in Y:
        line_intensity = []
        for x in X:
            line_intensity.append(wsn_rate[(x,y)])
        intensity.append(line_intensity)
    z = np.array(intensity)
    wifi = wifis[0]        
    ws_img_temp = wifi.ws_img.copy()
    # -------- plot
    figure = plt.figure()
    ax = figure.add_subplot(1,1,1)    
    # draw workspace image
    im = plt.imshow(z, extent=(0, len(ws_img_temp[0]), len(ws_img_temp), 0),interpolation='hanning', origin = 'upper', cmap=cm.hot,zorder=2,alpha=0.9)
    ax.imshow(ws_img_temp, zorder=1, origin='upper')
    #ax.axis('image')
    plt.axis('off')
    # draw wifis and sink
    for wifi in wifis:
        loc = wifi.loc
        ax.plot(loc[0], loc[1], color='c', marker='p',fillstyle='full', zorder=3,markersize=10)
    if sink:
        loc = sink.loc
        ax.plot(loc[0], loc[1], color='m', marker='D',fillstyle='full', zorder=3,markersize=8)
    #add_colorbar()
    plt.colorbar(im,fraction=0.046, shrink=0.55)
    plt.savefig(file_name, bbox_inches='tight')
    print '---%s saved---' %file_name


    
if __name__ == '__main__':
    ws_img_dir ='./wsBW.png'
    # wifi_one
    wifi_one_loc = (850, 20)
    wifi_one = wifi_2D(wifi_one_loc, ws_img_dir)
    # wifi_two
    wifi_two_loc = (100, 425)
    wifi_two = wifi_2D(wifi_two_loc, ws_img_dir)
    # wifi_three
    wifi_three_loc = (700, 425)
    wifi_three = wifi_2D(wifi_three_loc, ws_img_dir)
    # wifi_four
    wifi_four_loc = (400, 20)
    wifi_four = wifi_2D(wifi_four_loc, ws_img_dir)    
    # sink 
    sink_loc = (20, 20)
    sink = wifi_2D(sink_loc, ws_img_dir)
    # source
    source = wifi_2D()
    # compose
    wifis = [wifi_one, wifi_two, wifi_three, wifi_four]
    # compose_wifi_rate(wifis, source, sink, './rate_map.p', 50)
    draw_wsn_rate('./rate_map.p', wifis, sink, file_name='wsn_rate.pdf')
