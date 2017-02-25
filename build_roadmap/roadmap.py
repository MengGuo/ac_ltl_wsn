#!/usr/bin/env python

import sys, random, math
from math import sqrt, cos, sin, atan2, floor, fabs
from numpy import sign

from scipy.misc import imread

from networkx.classes.digraph import Graph

import pickle

import matplotlib
from matplotlib import cm
import matplotlib.pyplot as plt

matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True


class roadmap_2D(Graph):
    '''
    Construct a 2D roadmap via loading a BW workspace .png file
    Use sampling based method.    
    '''
    def __init__(self, ws_img_dir='./wsBW.png', pixel_meter_ratio=1):
        img = imread(ws_img_dir)
        pickle.dump(img, open('./ws_img.p', "wb"))
        Graph.__init__(self, name='roadmap_2D', ws_img=img)
        self.maxX = len(img[0])-1
        self.maxY = len(img)-1
        self.pixel2meter = pixel_meter_ratio
        print '----roadmap_2D initialized, x: %d y:%d---' %(self.maxX, self.maxY)


    def construct_roadmap(self, total_sample_number, max_step_distance, file_name='roadmap_2D.p'):
        # max_step_distance in meter
        self.max_step = max_step_distance
        while (len(self.nodes()) < total_sample_number):
            sample_point = self.generate_free_sample()
            near_nodes = self.find_near_nodes(sample_point, max_step_distance)
            # control distribution density of the samples
            rho1 = 0.0008
            rho2 = 0.35
            if self.nodes():
                min_p = min(self.nodes(), key=lambda p: self.point_dist(p, sample_point))
                min_dist = self.point_dist(min_p,sample_point)
            else:
                min_dist = max_step_distance
            if ((len(near_nodes) < rho1*max_step_distance**2) and (min_dist> rho2*max_step_distance)):
                for near_node in near_nodes:
                    if (near_node != sample_point) and (self.point_dist(near_node, sample_point) > rho2*max_step_distance):
                        paths = self.steer(near_node, sample_point, max_step_distance)
                        self.add_new_nodes(paths,rho2)
                    else:
                        self.add_node(sample_point)
        print '---roadmap_2D construction done, size:%d nodes, %d edges---' %(len(self.nodes()), len(self.edges()))
        pickle.dump(self.edges(), open(file_name, "wb"))
        print '---roadmap_2D edges saved as %s---' %file_name


    def load_roadmap(self, file_dir='./roadmap_2D.p'):
        edges = pickle.load(open(file_dir, 'rb'))
        for e in edges:
            self.add_edge(e[0], e[1], weight=self.point_dist(e[0],e[1]))
        print '---roadmap_2D loaded from %s, %d nodes, %d edges---' %(file_dir, len(self.nodes()), len(self.edges()))

            
    def generate_free_sample(self):
        obstacle_occupied = True
        while obstacle_occupied:
            sample_x = random.randint(0, self.maxX)
            sample_y = random.randint(0, self.maxY)
            sample_point = (sample_x, sample_y)
            if ((not self.check_collision(sample_point)) and (sample_point not in self.nodes())):
                obstacle_occupied = False
        return sample_point


    def point_dist(self, p1, p2):
        # pixel dist to meter
        return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)*self.pixel2meter

    
    def find_near_nodes(self, testp, max_step):
        near_nodes = set()
        if len(self.nodes()) == 0:
            near_nodes.add(testp)
        else:
            near_nodes = set([f for f in self.nodes() if self.point_dist(f, testp) <= self.max_step])
            if len(near_nodes) == 0:
                nearp = min(self.nodes(), key=lambda p: self.point_dist(p, testp))
                near_nodes.add(nearp)
        return near_nodes

    
    def steer(self, node, point, max_step_distance):
        '''
        Straight line search via every max_distance
        '''
        paths = [node,]
        step_x = (point[0]-node[0])/float(max_step_distance)
        step_y = (point[1]-node[1])/float(max_step_distance)
        number_points_needed = int(floor(max([fabs(step_x), fabs(step_y)])))+1
        if fabs(step_x) > fabs(step_y):
            ratio = [sign(step_x), step_y/fabs(step_x)]
        else:
            ratio = [step_x/fabs(step_y), sign(step_y)]
        obstacle_occupied = False
        for i in range(number_points_needed):
            for j in range(max_step_distance):
                point_x = round(paths[i][0]+ratio[0]*j)
                point_y = round(paths[i][1]+ratio[1]*j)
                if ((point_y >= len(self.graph['ws_img'])) or (point_x >= len(self.graph['ws_img'][0]))):
                    break
                obstacle_occupied = self.check_collision((point_x, point_y))
                if obstacle_occupied:
                    break
                if ((point_x, point_y) == point):
                    break
            if obstacle_occupied:
                break
            if ((point_x, point_y) == point):
                break
            if ((point_x, point_y) != point):
                new_point_x =  round(paths[i][0]+(ratio[0]*max_step_distance))
                new_point_y =  round(paths[i][1]+(ratio[1]*max_step_distance))
                paths.append((new_point_x, new_point_y))
        if ((not obstacle_occupied) and (number_points_needed == 1)):
            paths.append(point)
        if (((point_x, point_y) == point) and (point not in paths)):
            paths.append(point)
        return paths

    
    def add_new_nodes(self, paths, rho):
        if len(paths) > 1:
            for l in range(len(paths)-1):
                self.add_edge(paths[l], paths[l+1], weight=self.point_dist(paths[l],paths[l+1]))
                near_nodes = set([f for f in self.nodes() if rho*self.max_step <= self.point_dist(f, paths[l]) <= self.max_step])
                for near_node in near_nodes:
                    line = self.steer(near_node, paths[l], self.max_step)
                    if len(line) >= 2:
                        self.add_edge(paths[l], near_node, weight=self.point_dist(paths[l],near_node))
            


    def check_collision(self, point):
        for x in [int(point[0]) + l for l in range(-4, 4)]:
            for y in [int(point[1]) + l for l in range(-4, 4)]:
                if ((y < len(self.graph['ws_img'])) and (x < len(self.graph['ws_img'][0]))):
                    if self.graph['ws_img'][y][x][0] < 255:
                        return True
        return False


    def draw(self, name='./roadmap_2D.pdf'):
        figure = plt.figure()
        ax = figure.add_subplot(1,1,1)
        # draw workspace image
        ax.imshow(self.graph['ws_img'], cmap=cm.Greys_r)
        #ax.axis('image')
        plt.axis('off')
        # draw roadmap
        for edge in self.edges_iter():
            from_node = edge[0]
            to_node = edge[1]
            ax.plot([from_node[0], to_node[0]], [from_node[1], to_node[1]], color='g', linestyle='--', linewidth=1.5, marker='o', mfc='r', fillstyle='full', markersize=5)
        plt.savefig(name,bbox_inches='tight',pad_inches = 0) 
        print '---roadmap saved as %s---' %name
        return figure
        
        
if __name__ == '__main__':
    map_img_dir ='./wsBW.png'
    roadmap = roadmap_2D(map_img_dir)
    # roadmap.construct_roadmap(150, 90, 'roadmap_2D.p')
    roadmap.load_roadmap('./roadmap_2D.p')
    roadmap.draw()


    
