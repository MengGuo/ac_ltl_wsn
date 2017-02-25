#!/usr/bin/env python

from math import sqrt, log, fabs, exp
from scipy.misc import imread
from numpy import sign, zeros


def dist_2D(p1, p2):
    return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


class wifi_2D(object):
    def __init__(self, wifi_loc=(0,0), ws_img_dir='./wsBW.png', rate=10, power=1, alpha=0.13):
        self.rate = rate
        self.power = power
        self.alpha = alpha
        self.ws_img = imread(ws_img_dir)
        self.loc = wifi_loc
        print '---Wifi model initialized---'

    def wifi_snr(self, point):
        # log-distance path loss model and take into account walls
        wsMap = self.ws_img
        maxY = len(wsMap)
        maxX = len(wsMap[0])
        if ((not (0<=point[0]<maxX)) or (not (0<=point[1]<maxX))):
            print 'provided pose outside the image'
        (point_x, point_y) = point
        dist = dist_2D(point, self.loc)
        if  (dist <= exp(self.power/self.alpha)):
            snr_point = self.power - self.alpha*log(dist+1)
            if dist >= 2:
                number_walls = self.count_walls(point)
                snr_point -= self.alpha*number_walls*0.05
            if snr_point < 0:
                snr_point = 0
        else:
            snr_point = 0
        return snr_point
    
    def count_walls(self, point):
        number_walls = 0
        wifi_loc = self.loc
        dif_x = int(point[0] - wifi_loc[0])
        dif_y = int(point[1] - wifi_loc[1])
        if fabs(dif_x) > fabs(dif_y):
            ratio = [sign(dif_x), dif_y/fabs(dif_x)]
        else:
            ratio = [dif_x/fabs(dif_y), sign(dif_y)]
        for k in range(int(max([fabs(dif_x), fabs(dif_y)]))):
            point_x = int(round(wifi_loc[0]+ratio[0]*k))
            point_y = int(round(wifi_loc[1]+ratio[1]*k))
            newpoint = (point_x, point_y)
            if (fabs(newpoint[0]-point[0]) > 2):
                if self.ws_img[point_y][point_x][0] < 255:
                    number_walls += 1
            else:
                return number_walls
        return number_walls


        
if __name__ == '__main__':
    ws_img_dir ='./wsBW.png'
    wifi_loc = (680, 380)
    router = wifi_2D(wifi_loc, ws_img_dir)
    point = (100, 100)
    point_snr = router.wifi_snr(point)
    print 'point snr at %s is %s' %(str(point), str(point_snr))
    # config_1 wifi_one, (400, 988), wifi_two, (435, 280), sink (400, 10)
    # config_2, wifi_one (10, 900), wifi_two, (435, 100), sink (435, 280)
    #router.load_heat_map(wifi_loc, img, 'wifi_two_heat_map.p')

    
