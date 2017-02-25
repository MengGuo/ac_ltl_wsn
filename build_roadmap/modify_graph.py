#!/usr/bin/env python

from scipy.misc import imread
import scipy.misc

map_img='./wsBW_original.png'
img = imread(map_img)

for x in range(50, 100):
    for y in range(600, 730):
        img[y][x][0:3] = [255, 255, 255]


for x in range(190, 320):
    for y in range(350, 400):
        img[y][x][0:3] = [255, 255, 255]

for x in range(190, 320):
    for y in range(580, 630):
        img[y][x][0:3] = [255, 255, 255]

for x in range(320, 380):
    for y in range(680, 730):
        img[y][x][0:3] = [255, 255, 255]        


for x in range(120, 180):
    for y in range(280, 330):
        img[y][x][0:3] = [255, 255, 255]

for x in range(10, 120):
    for y in range(230, 260):
        img[y][x][0:3] = [255, 255, 255]

for x in range(10, 200):
    for y in range(120, 160):
        img[y][x][0:3] = [255, 255, 255]

for x in range(100, 120):
    for y in range(10, 75):
        img[y][x][0:3] = [255, 255, 255]

for x in range(205, 260):
    for y in range(205, 270):
        img[y][x][0:3] = [255, 255, 255]

for x in range(110, 160):
    for y in range(200, 230):
        img[y][x][0:3] = [255, 255, 255]

for x in range(180, 340):
    for y in range(100, 140):
        img[y][x][0:3] = [255, 255, 255]

for x in range(10, 200):
    for y in range(50, 150):
        img[y][x][0:3] = [255, 255, 255]

for x in range(85, 130):
    for y in range(150, 200):
        img[y][x][0:3] = [255, 255, 255]

for x in range(80, 100):
    for y in range(150, 180):
        img[y][x][0:3] = [255, 255, 255]

for x in range(230, 280):
    for y in range(480, 530):
        img[y][x][0:3] = [255, 255, 255]                        

scipy.misc.imsave('mapBW_modified.png', img)
        
