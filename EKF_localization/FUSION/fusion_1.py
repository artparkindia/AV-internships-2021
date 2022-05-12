#############################################
#Lidar GPS fusion -  EKF
#Uses ICP output of lidar point cloud and GPS converted coordinates in the Measurement Model

#LIDAR - X,Y
#GPS - X,Y
#############################################

import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd
import itertools
import toml


#Read the TOML config file
params = toml.load("fusion_config.toml")
basedir = params.get("basedir")
date = params.get("date")
drive = params.get("drive")

dataset = pykitti.raw(basedir, date, drive)
oxts = dataset.oxts
time_stamp = dataset.timestamps
N = len(oxts) - 3

# Extract pre-generated LIDAR ICP and GPS data
pose_lidar = np.array(np.loadtxt('LIDAR_' + date + '_' + drive + ".txt"))
pose_gps = np.array(np.loadtxt('GPS_' + date + '_' + drive + ".txt"))


#Process noise parameters
sigma_ax2 = params.get("sigma_ax2")
sigma_ay2 = params.get("sigma_ay2")
#Measurement noise parameters: Lidar
sigma_x2_lid = params.get("sigma_x2_lid")
sigma_y2_lid = params.get("sigma_y2_lid")
#Measurement noise parameters: GPS
sigma_x2_gps = params.get("sigma_x2_gps")
sigma_y2_gps = params.get("sigma_y2_gps")


#initialization
P = np.asarray([[10,0,0,0],[0,10,0,0],[0,0,100,0],[0,0,0,100]])
X = np.asarray([[1],[1],[1],[0.0]])


#Lidar
HLidar = np.asarray([[1,0,0,0],[0,1,0,0]])
#noise covariance
RLidar = np.asarray([[sigma_x2_lidar,0],[0,sigma_y2_lidar]]) 

#gps
HGps = np.asarray([[1,0,0,0],[0,1,0,0]])
RGps = np.asarray([[sigma_x2_gps,0],[0,sigma_y2_gps]]) 

#States
xpos=[]
ypos=[]
vx=[]
vy=[]

#Measurements
xpos_lidar=[]
ypos_lidar = []
xpos_gps=[]
ypos_gps = []

#number of parameters measured
Lidarstates = 2
Gpsstates = 2


def predict(X,P):
        F = Fmatrix(dt)
        X = np.matmul(F,X)
        P = np.add(np.matmul(np.asarray(np.matmul(F,P)),np.transpose(F)),Q(dt))
        return X,P
    
def correct(X,P,Z, sensor_states, H_sensor,R_sensor):
        
        S = H_sensor @ P @ H_sensor.T + R_sensor
        K = P @ H_sensor.T @ np.linalg.inv(S)
        y = np.reshape(np.asarray(Z[0:sensor_states]),(sensor_states,1))- H_sensor @ X
        X = X + K @ y
        P = np.matmul(np.identity(len(X)) - np.matmul(K,H_sensor),P)
        return X,P
    

#Linear Motion Model
def Fmatrix(dt):
    return np.asarray([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
 
#Process covariance
def Q(dt):
     return np.asarray([[sigma_ax2*dt**4/4,0,sigma_ax2*dt**3/2,0],
                        [0,sigma_ay2*dt**4/4,0,sigma_ay2*dt**3/2],
                        [sigma_ax2*dt**3/2,0,sigma_ax2*dt**2,0],
                        [0,sigma_ay2*dt**3/2,0,sigma_ay2*dt**2]])
    
for i in range(0,N):
    
    dt = (time_stamp[i].microsecond - time_stamp[i-1].microsecond)/10000.0  #10,000
    
    # copy data
    xpos_lidar.append(pose_lidar[i][0])
    ypos_lidar.append(pose_lidar[i][1])
    xpos_gps.append(pose_gps[i][0])
    ypos_gps.append(pose_gps[i][1])
    
    # EKF
    X,P = predict(X,P)
    X,P = correct(X,P,pose_gps[i],Gpsstates,HGps,RGps)
    X,P = correct(X,P,pose_lidar[i],Lidarstates,HLidar,RLidar) 
   
    # Update states
    xpos.append(X[0])
    ypos.append(X[1])
    vx.append(X[2])
    vy.append(X[3])
    
plt.figure(figsize=(12,8))
plt.plot(xpos_lidar,ypos_lidar,label='Measured by Lidar', color = 'g')
plt.plot(xpos_gps,ypos_gps,label='Measured by GPS',color='b')
plt.plot(xpos,ypos,label='EKF - LIDAR+GPS',color='r',ls='--')
plt.xlabel('X-coordinate')
plt.ylabel('Y-coordinate')
plt.legend()
plt.show()
