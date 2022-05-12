#GPS Data extraction from KITTI and transformation using Mercator Scale

import pykitti
import math
import numpy as np
import toml


#Read the TOML config file
params = toml.load("gps_config.toml")
basedir = params.get("basedir")
date = params.get("date")
drive = params.get("drive")


dataset = pykitti.raw(basedir, date, drive)
oxts = dataset.oxts

a = 6378137.0
b = 6356752.3142
e = math.sqrt(1-(b**2/a**2))

x_gps = []
y_gps = []
z_gps = []
yaw_gps = []
p = []

pose_gps = []

def gps_raw_to_xyz(lat,long,alt):
    return s*a*(np.pi*long/180) , s*a*np.log(np.tan(np.pi*(90+lat)/360)) , alt


lat_0 =  oxts[0][0][0]
long_0 = oxts[0][0][1]
alt_0 = oxts[0][0][2]
    
phi = oxts[0][0][3]
theta = oxts[0][0][4]
psi = oxts[0][0][5]
    
s = np.cos(lat_0*np.pi/180)
x_0 =  s*a*(np.pi*long_0/180)
y_0 = s*a*np.log(np.tan(np.pi*(90+lat_0)/360))
z_0 = alt_0
    
Tr_0_inv = np.linalg.inv(np.asarray([[np.cos(theta)*np.cos(psi),np.cos(psi)*np.sin(phi)*np.sin(theta) - np.cos(phi)*np.sin(psi),np.cos(phi)*np.cos(psi)*np.sin(theta) + np.sin(theta)*np.sin(psi),x_0],
                   [np.cos(theta)*np.sin(psi),np.cos(psi)*np.cos(phi) + np.sin(phi)*np.sin(psi)*np.sin(theta),-np.sin(phi)*np.cos(psi) + np.sin(theta)*np.sin(psi)*np.cos(phi),y_0],
                   [-np.sin(theta),np.cos(theta)*np.sin(phi),np.cos(phi)*np.sin(theta),z_0],
                   [0,0,0,1]]))
    
    
for i in range(0,len(oxts)):
    lat = oxts[i][0][0]
    long = oxts[i][0][1]
    alt = oxts[i][0][2]
    
    x,y,z = gps_raw_to_xyz(lat,long,alt)
 
    p_g = np.array([x,y,z,1])
    p = np.dot(Tr_0_inv, p_g)
    x_gps.append(p[0])
    y_gps.append(p[1])
    z_gps.append(p[2])
    pose_gps.append(p)


# Saving the Lidar pose in a text file
filename = "GPS_" + date + "_" + drive
np.savetxt(filename + ".txt", pose_gps)
    
plt.figure(figsize = (12,8))
plt.plot(x_gps,y_gps,color = 'r')
plt.xlabel('X-coordinate')
plt.ylabel('Y-coordinate')
plt.show()
