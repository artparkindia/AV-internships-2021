# -*- coding: utf-8 -*-
"""
Created on Sun Jun  6 19:39:18 2021

@author: Awies Mohammad Mulla
"""
import numpy as np
import random
import matplotlib.pyplot as plt
import pandas as pd
import cv2

"""
Dimensions of image imported:
   width = 789
   height= 491
   RGBA color channel
Make appropriate changes to the code if using custom image.
"""

#Import the image to set as background
img = cv2.imread('maps/map_3_graph.png')
#Import the groundtruth poses of the car
data = pd.read_csv("Readings/Readings_3.csv")
data = np.array(data)
#Import the signed distances calculated
fnfd = pd.read_csv("Distances/Distances_3.csv")
fnfd = np.array(fnfd)
#List of X Y coordinates and corresponding colors
X = []
Y = []
C = []
#Iterate through different poses
for n in range(len(data)):
    #list of odom coordinates
    x = []
    y = []
    c = []
    #list of LiDAR points
    lid_x = []
    lid_y = []
    #gpose is groundtruth
    #state is mean of prior (odom distribution)
    state = data[n]
    gpose = data[n]
    
    #Rotation matrix to align the distribution plot with thw road
    alpha = (state[3] * (-1.5))  #angular pose of car
    rot = np.array([[np.cos(np.radians(alpha)) , np.sin(np.radians(alpha))],
                    [-np.sin(np.radians(alpha)) , np.cos(np.radians(alpha))]])
    #create an error from groundtruth after certain timestep
    if (n % 7) == 0:
        state[1] = random.uniform((state[1] - (10 * np.cos(np.radians(state[3])))),
                                  (state[1] + (10 * np.cos(np.radians(state[3])))))
        state[2] = random.uniform((state[2] - (10 * np.sin(np.radians(state[3])))),
                                  (state[2] + (10 * np.sin(np.radians(state[3])))))
    #Generating different possible poses around mean pose
    for i in range(30):
        for j in range(30):
            x.append(state[1] + j)
            x.append(state[1] + j)
            y.append(state[2] + i)
            y.append(state[2] - i)
        for j in range(30):
            x.append(state[1] - j)
            x.append(state[1] - j)
            y.append(state[2] + i)
            y.append(state[2] - i)
    #Align the generated poses to road using rotation matrix
    for i in range(len(x)):
        temp = np.array([x[i] , y[i]])
        pose = np.array([state[1] , state[2]])
        temp = np.dot((temp - pose) , rot) + temp
        x[i] = temp[0]
        y[i] = temp[1]
    #Obtaining possible LiDAR points
    for i in range(30):
        for j in range(30):
            lid_x.append(gpose[1] + j)
            lid_x.append(gpose[1] + j)
            lid_y.append(gpose[2] + i)
            lid_y.append(gpose[2] - i)
        for j in range(30):
            lid_x.append(gpose[1] - j)
            lid_x.append(gpose[1] - j)
            lid_y.append(gpose[2] + i)
            lid_y.append(gpose[2] - i)
    #Transforming the LiDAR points along the road 
    for i in range(len(lid_x)):
        temp = np.array([lid_x[i] , lid_y[i]])
        pose = np.array([gpose[1] , gpose[2]])
        temp = np.dot((temp - pose) , rot) + temp
        lid_x[i] = temp[0]
        lid_y[i] = temp[1]    
    #marking points currently detected from LiDAR in look up table 
    counter = np.zeros((491 , 789))
    for i in range(len(lid_x)):
        temp1 = np.int(lid_x[i])
        temp2 = np.int(lid_y[i])
        if temp1 >= 789:
            temp1 = 788
        if temp2 >= 491:
            temp2 = 490
        counter[temp2][temp1] = counter[temp2][temp1] + 1        
    #Genrating the distribution using prior and likelihood
    for i in range(len(x)):
        #Error between mean pose and possible generated pose
        delta_x   = abs(x[i]-state[1])
        delta_y   = abs(y[i]-state[2])
        delta_phi = abs((random.uniform(np.radians(state[3])-0.05,np.radians(state[3])+0.1)))
        #Calculate the prior
        #(prior is gaussian distribution)
        temp = np.array([delta_x, delta_y, delta_phi])
        #30 mentioned is the scaling parameter (change in gradient across the coordinates)
        odom = (2.71828 ** ((-np.linalg.norm(temp))/30)) 
        #Check whether LiDAR scan present for current pose 
        #If present, use it to calculate the prob of car at current pose
        #And show it using different color channels        
        temp1 = np.int(x[i])
        temp2 = np.int(y[i])
        if temp1 >= 789:
            temp1 = 788
        if temp2 >= 491:
            temp2 = 490
        if counter[temp2][temp1]:
            if counter[temp2][temp1] > 0:
                counter[temp2][temp1] = -1 * counter[temp2][temp1]
            fd = fnfd[temp2][temp1 + 1]
            #25 mentioned is the scaling parameter related to road width (r_w)->(can take ~(r_w / 2))
            fd = (2.71828 ** (fd - 25))
            fd = fd + 1
            fd = 1 / (fd) 
            c.append(np.array([odom, fd, 0, 1]))   #if present, added two color channels                 
        else:
            c.append(np.array([odom, 0, 0, 1]))    #else, only used for prior
    #Plot the remaining LiDAR points which were not present in odom    
    for i in range(len(lid_x)):
        temp1 = np.int(lid_x[i])
        temp2 = np.int(lid_y[i])
        if temp1 >= 789:
            temp1 = 788
        if temp2 >= 491:
            temp2 = 490
        if counter[temp2][temp1] < 0:
            continue
        else:
            fd = fnfd[temp2][temp1 + 1]
            #25 mentioned is the scaling parameter related to road width (r_w)->(can take ~(r_w / 2))
            fd = (2.71828 ** (fd - 25))
            fd = fd + 1
            fd = 1 / (fd) 
            x.append(lid_x[i])
            y.append(lid_y[i])
            c.append(np.array([0, fd, 0, 1]))    #one channel used for LiDAR scan  
    #Add the poses and corresponding colors to the list
    X.append(np.array(x))
    Y.append(np.array(y))
    C.append(np.array(c))
    print(n)

for i in range(len(X)):
    #No change in pose of car so ignored
    if i <= 40:
        continue
    #plot the points from above list
    plt.scatter(X[i],Y[i],s=1,c=C[i])
    plt.imshow(img, zorder=0)
    #save each state
    #use appropriate folder corresponding to map used
    plt.savefig('output/sim_3/test%d.png'%(i))
    #Clear screen after each state
    plt.clf()
    print(i)
        
#To compile the images saved use the ffmpeg command mentioned below:

#ffmpeg -r 35 -f image2 -s 1920x1080 -start_number 61 -i test%d.png -vcodec libx264 -crf 25  -pix_fmt yuv420p test.mp4            
