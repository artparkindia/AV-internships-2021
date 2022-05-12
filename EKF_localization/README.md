# ARTPARK-Internship-2021

## Sensor Fusion for Robot Localization

### Repository contents

1. EKF on a tracking problem using LIDAR and RADAR from Udacity course, improvised with Exponentially weighted EKF

2. GPS Data extraction from KITTI and Conversion using Mercator scale

3. LIDAR Point cloud extraction from KITTI ; ICP Implementation and Transformation for Pose estimation

4. Simple LIDAR + GPS Fusion (KITTI)

5. LIDAR + GPS + IMU Fusion - Non-Quaternion formulation (KITTI)

6. LIDAR + GPS + IMU  Fusion - Quaternion formulation (KITTI)

7. LIDAR + DMI + IMU Fusion - Quaternion formulation (IISc dataset)


### Dependencies

1. PyKITTI -     ```pip install pykitti```

2. TOML - ```pip install toml```


#### Steps to run are provided in the [FUSION](https://github.com/naveenmoto/artpark-internship-2021/tree/main/Atreya_Bhat/FUSION) Folder


### KITTI DATASETS FOLDER STRUCTURE


![kitti_folder_struct](https://user-images.githubusercontent.com/39030188/120890425-2f50a700-c620-11eb-8c8c-2c1c66895c53.png)


### Notes

ICP works well for smooth paths, Inaccurate for sharp turns

GPS Altitude output is directly used in the fusion without any conversion, might require a conversion/ offset for more accurate 3D Pose plots

NO Ground Truth data available with KITTI

