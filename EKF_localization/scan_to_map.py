#LIDAR ENCODER IMU FUSION 
# IISC datasets
# Vel_forward, Omega from ENCODER - Process
#X,Y,YAW from LIDAR - MM

import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd
import itertools
import toml
import pandas as pd
import sys
from IPython import display
import time
import copy


#Read the TOML config file
params = toml.load("fusion_config.toml")
# Extract pre-generated LIDAR ICP data
states = 5
path = "./iisc"


timestamps_lid = np.load(path + "/lidar/frame_time.npy")
imu = pd.read_csv(path + '/imu.csv',delimiter=',')
wheel = pd.read_csv( path + '/wheel_control.csv',delimiter=',')
imu_wheel = pd.merge_asof(left=imu , right=wheel, left_on='time', right_on='time')
pose_imu_enc = np.array(pd.merge_asof(left=pd.DataFrame(timestamps_lid, columns = ['time']) , right=imu_wheel, left_on='time', right_on='time'))

#Process noise parameters
sigma_ax2 = params.get("sigma_ax2")
sigma_ay2 = params.get("sigma_ay2")
#Measurement noise parameters: Lidar
sigma_x2_lid = params.get("sigma_x2_lid")
sigma_y2_lid = params.get("sigma_y2_lid")
sigma_yaw_lid = params.get("sigma_theta_lid")

#initialization
P = np.zeros((states,states)) 
np.fill_diagonal(P, 10)
X = np.asarray([[0],[0],[0],[0],[0]])

#Lidar
HLidar = np.asarray([[1,0,0,0,0],[0,1,0,0,0],[0,0,0,1,0]])
RLidar = np.asarray([[sigma_yaw_lid,0,0],[0,sigma_yaw_lid,0],[0,0,sigma_yaw_lid]]) 

# STATE MATRIX
xpos=[]
ypos=[]
vf=[]
yaw = []
omega = []


# ENCODER measurements for process model
omega_enc = []
vel_enc = []

#IMU measurements for process model
omega_imu = []
vel_f_imu = []

#Number of states
Lidarstates = 3 #x,y,theta
Encoderstates = 2


origin_x = 1349
origin_y = 1019
scale = 50
    

#Process covariance
def Q(state,dt):
    x,y,v,theta,w = tuple(np.concatenate(state))
    sigma = 0.5
    return np.asarray([[sigma,0,dt*sigma*np.cos(theta),0,0],
                       [0,sigma,dt*sigma*np.sin(theta),0,0],
                       [0,0,sigma,0,0],
                       [0,0,0,sigma,sigma*dt],
                       [0,0,0,0,sigma]])

    
def Fmatrix(state):
    x,y,v,theta,w = tuple(np.concatenate(state))
    return np.asarray([[1,0,-v*dt*np.sin(theta),dt*np.cos(theta),0],
                       [0,1,v*dt*np.cos(theta),dt*np.sin(theta),0],
                       [0,0,1,0,0],
                       [0,0,0,1,dt],
                       [0,0,0,0,1]])

def Bmatrix(state,sensor):
    x,y,v,theta,w = tuple(np.concatenate(state))
    
    if(sensor == "imu"):
        return np.asarray([[0.5*dt**2,0],
                       [0,0.5*dt**2,0],
                       [dt,0,0],
                       [0,0,dt],
                       [0,0,1]])

    if(sensor == "enc"):
        return np.asarray([[dt*np.cos(theta),0],
                       [dt*np.sin(theta),0],
                       [1,0],
                       [0,dt],
                       [0,1]])

def predict_imu(X,P):
        F = Fmatrix(X)
        B = Bmatrix(X,"imu")
        acc_imu = pose_imu_enc[i][1:3]
        u = np.asarray([acc_imu[0],acc_imu[1],pose_imu_enc[i][6]])  
        X = np.matmul(F,X)
        P = np.add(np.matmul(np.asarray(np.matmul(F,P)),np.transpose(F)),Q(X,dt))
        return X,P
    
def predict_enc(X,P):
    
        vel_enc = pose_imu_enc[i][11]
        omega_enc = pose_imu_enc[i][12]
        F = Fmatrix(X)
        B = Bmatrix(X,"enc")
        u = np.asarray([vel_enc, omega_enc]).T
        X = np.matmul(F,X)
        P = np.add(np.matmul(np.asarray(np.matmul(F,P)),np.transpose(F)),Q(X,dt))
        return X,P 
    
def correct_lidar(X,P,Z):
        S=np.matmul(np.matmul(HLidar,P),np.transpose(HLidar)) + RLidar  
        K = np.dot(np.matmul(P,np.transpose(HLidar)),np.linalg.inv(S))
        y=np.reshape(np.asarray(Z),(Lidarstates,1))-np.matmul(HLidar,X)
        X = np.add(X,np.dot(K,y))
        P = np.matmul(np.identity(len(X))-np.matmul(K,HLidar),P)
        return X,P
    
    
def to_pose_vector(pose_hc):
     return [pose_hc[0][3], pose_hc[1][3], np.arccos(pose_hc[0][0])]

def to_tfmatrix(pose):
    x, y, theta = pose
    return np.array([[np.cos(theta), -np.sin(theta),0, x],[np.sin(theta), np.cos(theta), 0 , y],[0, 0, 1 ,0],[0, 0, 0, 1]])


pose_lidar = np.eye(4)
init_t = np.eye(4)


N = len(pose_imu_enc)

for i in range(150,N-600):
    
    dt = pose_imu_enc[i+1,0] - pose_imu_enc[i,0]
    v_prev = pose_imu_enc[i-1][1] * (pose_imu_enc[i,0] - pose_imu_enc[i-1,0])
    
    #X,P = predict_imu(X,P)
    X,P = predict_enc(X,P)    
        
    if(i%2==0 and pose_imu_enc[i][11] > 0.1):
        
        curr_scan = np.asarray(o3d.io.read_point_cloud("./iisc/lidar_PCD/frame_" + str(i) + ".pcd").points)
        scan = []

        for j in range(0,curr_scan.shape[0]):
            xp = curr_scan[j][0] * scale + origin_x
            yp = curr_scan[j][1] * scale + origin_y
            scan.append([xp,yp,0])
        
        scan_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(scan))
    
   
        t = open3d_robust_icp(new_map , scan_pcd,  init_t, threshold=0.1)
        init_t = t
        pose_lidar = t @ pose_lidar
        #pose_lidar.append([pose[0,3],pose[1,3]])
    
        new_map = new_map.transform(t)
        
        X,P = correct_lidar(X,P, to_pose_vector(pose_lidar))
        
        print("Processing init frame " + str(i))
        
        
        
    xpos.append(X[0])
    ypos.append(X[1])
    vf.append(X[2])
    yaw.append(X[3])
    omega.append(X[4])
    display.clear_output(wait=True)


    
plt.figure(figsize=(12,8))
#plt.plot(pose_lidar[:,0],pose_lidar[:,1],label='Measured by Lidar', color = 'g')
plt.plot(xpos,ypos,label='EKF - Lidar+IMU+Encoder',color='r',ls='--')
plt.xlabel('X-coordinate')
plt.ylabel('Y-coordinate')
plt.legend()
plt.show()
