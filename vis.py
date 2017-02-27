import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.ticker as ticker
from matplotlib import cm

import matplotlib.ticker as plticker

import matplotlib.gridspec as gridspec

import numpy as np

from math import sqrt, log

matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True


def dist_2D(p1, p2):
    return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


def make_movie(crm, num_pics, Robot_status_log, Dijk_log, movie_name='ltl_ac_learn'):
    i = 0
    # time, pose, action, buffer, Learning_seg
    draw_epi = 0
    for k,robot_status in enumerate(Robot_status_log[draw_epi]):
        t = robot_status[0]
        if robot_status[4] != None:
            cri_seg = robot_status[4]
            dijk_log = [s for s in Dijk_log[cri_seg] if s[0]<=t]
            #dijk_paths = dijk_log[-1][1:3]
            dijk_paths = dijk_log[0][1:3]
        else:
            dijk_paths = None
        # file_name = './movie/frame%s.png' %i
        file_name = './movie/frame%s.pdf' %i
        draw_robot_pose(crm, Robot_status_log[draw_epi][0:(k+1)], dijk_paths, file_name)
        i += 1
        if i> num_pics:
            break
    print 'movie pictures saved at ./movie/'

    
def draw_robot_pose(crm, robot_status_log, dijk_paths, file_name):
    figure = plt.figure()
    # ax =  figure.add_axes([0.1, 0.2, 0.4, 0.4])
    ax = figure.add_subplot(1,1,1)
    # draw workspace image
    ax.imshow(crm.graph['ws_img'], cmap=cm.Greys_r)
    # ax.set_axis_off()
    # time, pose, action, buffer, Learning_seg, theta
    # draw robot_pose and data_ratio
    # print 'robot_status_log', robot_status_log
    # draw wifis and sink
    for loc in crm.graph['wifis']:
        ax.plot(loc[0], loc[1], color='c', marker='p',fillstyle='full', zorder=3,markersize=13)
    sink_loc = crm.graph['sink']
    ax.plot(sink_loc[0], sink_loc[1], color='m', marker='D',fillstyle='full', zorder=3,markersize=13)
    # draw crm
    for edge in crm.edges_iter():
        from_node = edge[0]
        to_node = edge[1]
        ax.plot([from_node[0], to_node[0]], [from_node[1], to_node[1]], color='g', linestyle='--', linewidth=1.5, marker='o', mfc='r', fillstyle='full', markersize=3,alpha=0.5)
    # draw path
    if dijk_paths:
        direct_shortest_path, arg_shortest_path = dijk_paths
        # print 'direct_shortest_path', direct_shortest_path
        # print 'arg_shortest_path', arg_shortest_path
        for l in range(len(direct_shortest_path)-1):
            from_node = direct_shortest_path[l]
            to_node = direct_shortest_path[l+1]            
            line1 = ax.plot([from_node[0], to_node[0]], [from_node[1], to_node[1]], color='r', linestyle='-', linewidth=3, markersize=1.5,label='dijk',zorder=4)
        # draw arg_shortest_path
        for l in range(len(arg_shortest_path)-1):
            from_node = arg_shortest_path[l][0]
            to_node = arg_shortest_path[l+1][0]
            line2 = ax.plot([from_node[0], to_node[0]], [from_node[1], to_node[1]], color='k', linestyle='-', linewidth=3, markersize=1.5,label='arg_dijk',zorder=4)
    robot_status = robot_status_log[-1]
    t = robot_status[0]
    (x, y) = robot_status[1]
    # for p_robot_status in robot_status_log:
    #     (p_x, p_y) = p_robot_status[1]
    #     ax.plot(p_x, p_y, color='b', marker='o',fillstyle='full', zorder=3,markersize=12)
    for l in range(len(robot_status_log)-1):
        from_node = robot_status_log[l][1]
        to_node = robot_status_log[l+1][1]            
        robot_line = ax.plot([from_node[0], to_node[0]], [from_node[1], to_node[1]], color='y', linestyle='-', linewidth=3, markersize=1.5, label='LSTD_ac',zorder=3)
    action = robot_status[2]
    if action != None:
        act_text = ax.text(x, y-20, r'$%s$'%action, fontsize=12, zorder=3, fontweight ='bold')
        act_text.set_bbox(dict(color='red', alpha=0.7, edgecolor='red'))
        ax.set_title('Time: %d. Perform action %s' %(int(t), action))
    robot_data_ratio = robot_status[3][0]*1.0/robot_status[3][1]
    (dx, dy) = (43, 10)
    (lx, ly) = (10, 32)
    data_rec_current = matplotlib.patches.Rectangle((x-dx, y-dy), lx*2, ly*2*robot_data_ratio, fill = True, facecolor = 'magenta', edgecolor = 'black', linewidth = 1,  alpha =0.8, zorder=5)
    ax.add_patch(data_rec_current)
    data_rec_full = matplotlib.patches.Rectangle((x-dx, y-dy), lx*2, ly*2, fill = False, edgecolor = 'black', linewidth = 1,  alpha =0.8, zorder=5)
    ax.add_patch(data_rec_full)
    ax.plot(x, y, color='b', marker='o',fillstyle='full', zorder=5,markersize=12)
    # draw initial and goal
    pixel_to_meter = 0.1
    if robot_status[4] != None:
        start = robot_status[4][0][0]
        goal = robot_status[4][1][0]
        ax.plot(start[0], start[1], color='r', marker='s',fillstyle='full', zorder=5,markersize=13)
        ax.plot(goal[0], goal[1], color='r', marker='*',fillstyle='full', zorder=5,markersize=13)
        if robot_status[5] != None:
            theta1, theta2 = robot_status[5]
            ax.set_title(r'Time: %d. Segment $(%s, %s): \theta_1=%.2f,\; \theta_2=%.2f$' %(int(t), str((int(start[0]*pixel_to_meter),int(start[1]*pixel_to_meter))),str((int(goal[0]*pixel_to_meter),int(goal[1]*pixel_to_meter))),theta1,theta2))
        else:
            ax.set_title(r'Time: %d. Motion to $(%s, %s)$' %(int(t), str((int(start[0]*pixel_to_meter),int(start[1]*pixel_to_meter))),str((int(goal[0]*pixel_to_meter),int(goal[1]*pixel_to_meter)))))
    else:
         ax.set_title(r'Time: %d. Robot at $%s$' %(int(t), str((int(x*pixel_to_meter),int(y*pixel_to_meter)))))
    ax.set_xticks([0,200,400,600,800])
    ax.set_xticklabels(['0','20','40','60','80'])
    ax.set_yticks([0,100,200,300,400])
    ax.set_yticklabels(['0','10','20','30','40'])
    ax.set_xlim([0,989])
    ax.set_ylim([444,0])
    # --- for .png
    # DPI = 1500
    # plt.savefig(file_name,bbox_inches='tight',dip=DPI)
    # --- for .pdf
    plt.savefig(file_name,bbox_inches='tight')
    plt.close()


def draw_data(Robot_status_log, file_name='./data/ltl_ac_buffer.pdf'):
    figure = plt.figure()
    ax = figure.add_subplot(1,1,1)
    # time, pose, action, buffer, Learning_seg
    time_seq = [s[0] for epi_s in Robot_status_log for s in epi_s]
    data_seq1 = [s[3][0] for epi_s in Robot_status_log for s in epi_s]
    data_seq2 = [s[3][1] for epi_s in Robot_status_log for s in epi_s]
    ax.plot(time_seq, data_seq1, color='b', linestyle='-', linewidth=3,label=r'$b(t)$')
    ax.plot(time_seq, data_seq2, color='r', linestyle='-', linewidth=3,label=r'$\overline{B}$')
    ax.set_xlabel(r'$Time(s)$')
    ax.set_ylabel(r'$Buffer\;Size\; (Mb)$')
    plt.savefig(file_name, bbox_inches='tight')
    print '%s plotted and saved' %file_name

    
def draw_theta(Learn_log, file_name='./data/ltl_ac_theta.pdf'):
    figure = plt.figure()
    k = 0
    K = 3 # plot 3 cri_seg
    for cri_seg, learn_log in Learn_log.iteritems():
        if k <= 3:
            time_seq = []
            theta_seq = []
            for s in learn_log:
                time_seq.extend(list([s[0],s[0],s[1]]))
                theta_seq.extend(list([s[2][0][2][0][1],
                                  s[2][-1][2][-1][1],
                                       s[3][2][-1][1]]))
            k += 1
        theta1_seq = [s[0] for s in theta_seq]
        theta2_seq = [s[1] for s in theta_seq]
        ax = figure.add_subplot(2,2,k)
        ax.plot(time_seq, theta1_seq, color='b', linestyle='-', linewidth=3,label=r'$\theta_1$')
        ax.plot(time_seq, theta2_seq, color='r', linestyle='--', linewidth=3,label=r'$\theta_2$')
        ax.set_xlim(min(time_seq)-10,max(time_seq)+1)
        ax.set_ylim(0, max([max(theta1_seq),max(theta2_seq)])+0.3)
        # ax.set_xlabel(r'$Time(s)$')
        # ax.set_ylabel(r'${\theta}=[\theta_1,\;\theta_2]$')
        ax.legend(ncol=2,loc='center right')
    figure.text(0.5, 0.04, r'$Time(s)$', ha='center', va='center')
    figure.text(0.06, 0.5, r'${\theta}=[\theta_1,\;\theta_2]$', ha='center', va='center', rotation='vertical')
    #figure.tight_layout()
    plt.savefig(file_name, bbox_inches='tight')
    print '%s plotted and saved' %file_name

    
def draw_three_paths(crm, all_learn_log, direct_shortest_path, arg_shortest_path, file_name='./data/one_cri_seg_three_paths.pdf'):
    indirect_learn_log, direct_learn_log = all_learn_log[-1]
    traj_log, cost_log, theta_log, d_theta_log = direct_learn_log
    #--------------------
    figure = plt.figure()
    ax = figure.add_subplot(1,1,1)
    # draw workspace image
    ax.imshow(crm.graph['ws_img'], cmap=cm.Greys_r)
    # ax.set_axis_off()
    # draw initial and goal
    init = crm.graph['init']
    goal = crm.graph['goal']
    ax.plot(init[0], init[1], color='r', marker='s',fillstyle='full', zorder=3,markersize=13)
    ax.plot(goal[0], goal[1], color='r', marker='*',fillstyle='full', zorder=3,markersize=13)
    # draw wifis and sink
    for loc in crm.graph['wifis']:
        ax.plot(loc[0], loc[1], color='c', marker='p',fillstyle='full', zorder=3,markersize=13)
    sink_loc = crm.graph['sink']
    ax.plot(sink_loc[0], sink_loc[1], color='m', marker='D',fillstyle='full', zorder=3,markersize=13)
    # draw crm
    for edge in crm.edges_iter():
        from_node = edge[0]
        to_node = edge[1]
        ax.plot([from_node[0], to_node[0]], [from_node[1], to_node[1]], color='g', linestyle='--', linewidth=1.5, marker='o', mfc='r', fillstyle='full', markersize=3, alpha=0.7)
    # draw robot traj
    for l in range(len(traj_log)-1):
        from_node = traj_log[l][0]
        to_node = traj_log[l+1][0]
        line0 = ax.plot([from_node[0], to_node[0]], [from_node[1], to_node[1]], color='b', linestyle='-', linewidth=3, markersize=1.5,label='ac\_traj')
    # draw direct_shortest_path
    for l in range(len(direct_shortest_path)-1):
        from_node = direct_shortest_path[l]
        to_node = direct_shortest_path[l+1]
        line1 = ax.plot([from_node[0], to_node[0]], [from_node[1], to_node[1]], color='r', linestyle='-', linewidth=3, markersize=1.5,zorder=3)
    # draw arg_shortest_path
    for l in range(len(arg_shortest_path)-1):
        from_node = arg_shortest_path[l][0]
        to_node = arg_shortest_path[l+1][0]
        line2 = ax.plot([from_node[0], to_node[0]], [from_node[1], to_node[1]], color='k', linestyle='-', linewidth=3, markersize=1.5)        
    #plt.legend((line0, line1, line2), (r'$Ac\_traj$', r'$Dijk$', r'$arg\_Dijk$'), loc='lower center', ncol = 3, fontsize = 'x-small')
    ax.set_xticks([0,200,400,600,800])
    ax.set_xticklabels(['0','20','40','60','80'])
    ax.set_yticks([0,100,200,300,400])
    ax.set_yticklabels(['0','10','20','30','40'])
    plt.savefig(file_name, bbox_inches='tight')
    plt.clf()
    return figure    

def draw_score(MC_log, file_name='./data/ltl_ac_MC_score.pdf'):
    figure = plt.figure()
    ax = figure.add_subplot(1,1,1)
    ac_x = []
    ac_y = []
    ac_erry_lower = []
    ac_erry_higher = []
    dijk_x = []
    dijk_y = []
    dijk_erry_lower = []
    dijk_erry_higher = []
    k = 1
    for cri_seg, score in MC_log.iteritems():
        dijk_score, ac_score = score[0:2]
        full_dijk_score, dijk_scale = dijk_score
        full_ac_score, ac_scale = ac_score
        [l_ac_min, l_ac_ave, l_ac_max] = ac_scale[0:3]
        ac_x.append(k)
        ac_y.append(l_ac_ave)
        ac_erry_lower.append(l_ac_ave-l_ac_min)
        ac_erry_higher.append(l_ac_max-l_ac_ave)
        [l_dijk_min, l_dijk_ave, l_dijk_max] = dijk_scale[0:3]
        dijk_x.append(k)
        dijk_y.append(l_dijk_ave)
        dijk_erry_lower.append(l_dijk_ave-l_dijk_min)
        dijk_erry_higher.append(l_dijk_max-l_dijk_ave)
        k += 1
    ax.errorbar(ac_x, ac_y, yerr=[ac_erry_lower, ac_erry_higher],
            fmt='-s', color = 'g', ecolor='g', linewidth=3, capsize=3, capthick=3, label='LSTD Actor Critic')
    ax.errorbar(dijk_x, dijk_y, yerr=[dijk_erry_lower, dijk_erry_higher], fmt='--o', color= 'r', ecolor='r', linewidth=3, capsize=3, capthick=3, label='Adaptive Dijkstra')
    ax.legend(ncol=1, loc='best', labelspacing=1.1, numpoints=2, handlelength=2, prop={'size':15})
    ax.set_xlabel(r'$Critical\; Segments$')
    ax.set_ylabel(r'$Traj. \; Length$')
    plt.xticks(range(1,k))
    plt.savefig(file_name, bbox_inches='tight')
    plt.clf()
    return figure


def draw_init_plan(crm, Dijk_log, file_name = './data/ltl_ac_init_plan.pdf'):
    # Dijk_log[cri_seg].append([t, direct_shortest_path, arg_shortest_path])
    #--------------------
    figure = plt.figure()
    ax = figure.add_subplot(1,1,1)
    # draw workspace image
    ax.imshow(crm.graph['ws_img'], cmap=cm.Greys_r,)
    # ax.set_axis_off()
    # draw wifis and sink
    for loc in crm.graph['wifis']:
        ax.plot(loc[0], loc[1], color='c', marker='p',fillstyle='full', zorder=3,markersize=13)
    sink_loc = crm.graph['sink']
    ax.plot(sink_loc[0], sink_loc[1], color='m', marker='D',fillstyle='full', zorder=3,markersize=13)
    # draw crm
    for edge in crm.edges_iter():
        from_node = edge[0]
        to_node = edge[1]
        ax.plot([from_node[0], to_node[0]], [from_node[1], to_node[1]], color='g', linestyle='--', linewidth=1.5, marker='o', mfc='r', fillstyle='full', markersize=3, alpha=0.7)
    # for each cri_seg
    for cri_seg, dijk_log in Dijk_log.iteritems():
        init = cri_seg[0][0]
        goal = cri_seg[1][0]
        direct_shortest_path = dijk_log[-1][1]
        # draw initial and goal
        ax.plot(init[0], init[1], color='r', marker='*',fillstyle='full', zorder=3,markersize=13)
        ax.plot(goal[0], goal[1], color='r', marker='*',fillstyle='full', zorder=3,markersize=13)
        # draw direct_shortest_path
        for l in range(len(direct_shortest_path)-1):
            from_node = direct_shortest_path[l]
            to_node = direct_shortest_path[l+1]
            line1 = ax.plot([from_node[0], to_node[0]], [from_node[1], to_node[1]], color='b', linestyle='-', linewidth=3, markersize=1.5)
    ax.set_xticks([0,200,400,600,800])
    ax.set_xticklabels(['0','20','40','60','80'])
    ax.set_yticks([0,100,200,300,400])
    ax.set_yticklabels(['0','10','20','30','40'])
    plt.savefig(file_name, bbox_inches='tight')
    plt.clf()
    return figure    
