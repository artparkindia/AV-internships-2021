#LIDAR GPS IMU FUSION
#uses  vel_x, accel_X,accel_Y, Omega measurements from IMU
#X,Y, Theta from LIDAR
#X,Y,Z from GPS

import numpy as np
import pykitti
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
states = 11

# Extract pre-generated LIDAR ICP and GPS data
pose_lidar = np.array(np.loadtxt('LIDAR_' + date + '_' + drive + ".txt"))
pose_gps = np.array(np.loadtxt('GPS_' + date + '_' + drive + ".txt"))


#Process noise parameters
sigma_ax2 = params.get("sigma_ax2")
sigma_ay2 = params.get("sigma_ay2")
#Measurement noise parameters: Lidar
sigma_x2_lid = params.get("sigma_x2_lid")
sigma_y2_lid = params.get("sigma_y2_lid")
sigma_yaw_lid = params.get("sigma_theta_lid")
#Measurement noise parameters: GPS
sigma_x2_gps = params.get("sigma_x2_gps")
sigma_y2_gps = params.get("sigma_y2_gps")


#initialization
P = np.zeros((states,states)) 
np.fill_diagonal(P, 10)

X = np.asarray([[0],[0],[1],[1],[1],[1],[1],[1],[0.01],[0.01],[0.01]])

#GPS
HGps = np.asarray([[1,0,0,0,0,0,0,0,0,0,0],[0,1,0,0,0,0,0,0,0,0,0]])
RGps = np.asarray([[sigma_x2_gps,0],[0,sigma_y2_gps]]) 

#Lidar
HLidar = np.asarray([[1,0,0,0,0,0,0,0,0,0,0],[0,1,0,0,0,0,0,0,0,0,0],[0,0,1,0,0,0,0,0,0,0,0]])
RLidar = np.asarray([[sigma_x2_lid,0,0],[0,sigma_y2_lid,0],[0,0,sigma_yaw_lid]]) 


#STATE
xpos=[]
ypos=[]
vx=[]
vy = []
ax = []
ay = []
yaw = []
omega = []
abx = 0.01
aby = 0.01
b_omega = 0.01


# LIDAR Measurements
xpos_lidar=[]
ypos_lidar = []
yaw_lidar = []

#GPS Measurements
xpos_gps=[]
ypos_gps = []

# IMU measurements for process model
omega_imu = []
velx_imu = []
accelx_imu = []
accely_imu = []

#Number of states
Lidarstates = 3 #x,y,theta
Gpsstates = 2   #x,y

#Process covariance
def Q(state):

    return  dt**2 * np.diag([0.1,0.15,0.20,0.25,0.20,0.15,0.15,0.12,0.20,0.15,0.1])
    
       
def Fmatrix(dt):
    return np.asarray([[1,0,dt,0,0.5*dt**2,0,0,0,0,0,0],[0,1,0,dt,0,0.5*dt**2,0,0,0,0,0],[0,0,1,0,dt,0,0,0,0,0,0],[0,0,0,1,0,dt,0,0,0,0,0],
                       [0,0,0,0,1,0,0,0,0,0,0],[0,0,0,0,0,1,0,0,0,0,0],[0,0,0,0,0,0,1,dt,0,0,0],[0,0,0,0,0,1,0,1,0,0,0],
                       [0,0,0,0,0,0,0,0,total_time,0,0],[0,0,0,0,0,0,0,0,0,total_time,0],[0,0,0,0,0,0,0,0,0,0,total_time]])

def Bmatrix(dt):
    return np.asarray([[dt,0.5*dt**2,0,0],[0,0,0.5*dt**2,0],[1,dt,0,0],[0,0,dt,0],
                       [1/dt,1,0,0],[0,0,1,0],[0,0,0,dt],[0,0,0,1],
                       [0,0,0,0],[0,0,0,0],[0,0,0,0]])

    
def predict_with_imu(X,P):
        F = Fmatrix(dt)
        B = Bmatrix(dt)
        u = np.asarray([velx_imu[i],accelx_imu[i],accely_imu[i],omega_imu[i]])  
        X = np.matmul(F,X) + np.matmul(B,u).reshape(len(X),1)
        P = np.add(np.matmul(np.asarray(np.matmul(F,P)),np.transpose(F)),Q(dt))
        return X,P
    
    
def correct(X,P,Z, sensor_states, H_sensor,R_sensor):
        
        S = H_sensor @ P @ H_sensor.T + R_sensor
        K = P @ H_sensor.T @ np.linalg.inv(S)
        y = np.reshape(np.asarray(Z[0:sensor_states]),(sensor_states,1))- H_sensor @ X
        X = X + K @ y
        P = np.matmul(np.identity(len(X)) - np.matmul(K,H_sensor),P)
        return X,P
    
    
total_time = 0
for i in range(0,N):
    
    dt = (time_stamp[i].microsecond - time_stamp[i-1].microsecond)/100000.0    #1000000.0
    total_time = total_time + dt
    
    #copy measurements from dataset
    xpos_lidar.append(pose_lidar[i][0])
    ypos_lidar.append(pose_lidar[i][1])
    yaw_lidar.append(pose_lidar[i][2])
    xpos_gps.append(pose_gps[i][0])
    ypos_gps.append(pose_gps[i][1])
    omega_imu.append(oxts[i].packet.wz - b_omega*total_time)
    velx_imu.append(oxts[i].packet.vf)
    accelx_imu.append(oxts[i].packet.ax - abx*total_time)
    accely_imu.append(oxts[i].packet.ay - aby*total_time)

    
    X,P = predict_with_imu(X,P)
    if(oxts[i].packet.numsats > 3):  #if number of satellites < 3 and forward_vel ~= 0, reject GPS measurement
        X,P = correct(X,P,pose_gps[i][0:2],Gpsstates,HGps,RGps)
    X,P = correct(X,P,pose_lidar[i],Lidarstates,HLidar,RLidar)
    
    #update states
    xpos.append(X[0])
    ypos.append(X[1])
    vx.append(X[2])
    vy.append(X[3])
    ax.append(X[4])
    ay.append(X[5])
    yaw.append(X[6])
    omega.append(X[7])
    

plt.figure(figsize=(12,8))
plt.plot(xpos_lidar,ypos_lidar,label='Measured by Lidar', color = 'g')
plt.plot(xpos_gps,ypos_gps,label='Measured by GPS',color='b')
plt.plot(xpos,ypos,label='EKF - Lidar+GPS+IMU_Process',color='r',ls='--')
plt.xlabel('X-coordinate')
plt.ylabel('Y-coordinate')
plt.legend()
plt.show()
