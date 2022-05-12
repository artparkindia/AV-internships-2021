#ICP on LIDAR Point Cloud from KITTI in 2D
#Based on ClayFlannigan's ICP implementation - https://github.com/ClayFlannigan/icp

import pykitti
import toml
import sys
from IPython import display
import time
import numpy as np

def to_vector(pose_hc):
     return [pose_hc[0][2], pose_hc[1][2], np.arccos(pose_hc[0][0])] 

def to_hc(pose):
    x, y, theta = pose
    return np.array([[np.cos(theta), -np.sin(theta), x],[np.sin(theta), np.cos(theta), y],[0, 0, 1]])


#Read the TOML config file
params = (toml.load("icp_config.toml"))
basedir = params.get("basedir")
date = params.get("date")
drive = params.get("drive")
LIDAR_PTS = params.get("LIDAR_PTS") 
DIMS = params.get("DIMS")
tol = params.get("tolerance")

#Read KITTI dataset
dataset = pykitti.raw(basedir, date, drive)
time_stamps = dataset.timestamps
calib = dataset.calib.T_velo_imu

#Init
Num = len(time_stamps)-3
pose = to_hc([0, 0, 0])
icp_init = to_hc([0, 0, 0])
pose_lidar = []
total_time = 0

prev_frame = dataset.get_velo(0)[:LIDAR_PTS,:DIMS]
for i in range(0,Num):

    curr_frame = dataset.get_velo(i+1)[:LIDAR_PTS,:DIMS]
    
    start = time.time()
    T, _, iterations = icp(curr_frame,prev_frame, init_pose= icp_init, tolerance=tol)  #0.000001
    total_time += time.time() - start
    
    print("ICP progress " + str(i) + "/" + str(Num) + " frames , Time remaining = " + str(int(total_time*(Num-i)/(i+1))) + " Seconds")
    print("Last frame took " + str(iterations) + " Iterations")
    display.clear_output(wait=True)
    
    pose = T @ pose
    pose_lidar.append(to_vector(pose))
    prev_frame = curr_frame
    icp_init = T
    
# Saving the Lidar pose in a text file
filename = "LIDAR_" + date + "_" + drive
numpy.savetxt(filename + ".txt", pose_lidar)

print("Total time = " + str(int(total_time)) + " Seconds")
pose_lidar = np.array(pose_lidar)
plt.figure()
plt.plot(pose_lidar[:,0], pose_lidar[:,1])
plt.figure()
plt.plot(pose_lidar[:,2])
plt.show()
