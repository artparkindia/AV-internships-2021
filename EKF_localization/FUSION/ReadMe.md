The sensor fusion localisation is Benchmarked on KITTI dataset

## Walkthrough :

1. Import KITTI dataset and extract to different sensors data
2. For LIDAR pose est - Apply ICP on LIDAR PCD to get the vehicle pose plot in Global frame , the pose array is stored as a txt data file
3. For GPS pose est - Apply Mercator scale transformations on GPS to get vehicle pose in Global frame , the pose array is stored as a txt file
4. Fuse the data from multiple sensor measurements with the IMU data extracted directly using PyKITTI (NOT as txt file)
5. Multiple versions of fusion are formulated and the user is free to incorporate any of them as per their requirement


## Files 

1. Main python script with the EKF implementation

2. LIDAR- ICP and GPS Pose files, 
   - Sample txt files for a few dataset sequences available at [ICP](https://github.com/naveenmoto/artpark-internship-2021/tree/main/Atreya_Bhat/ICP) and [GPS](https://github.com/naveenmoto/artpark-internship-2021/tree/main/Atreya_Bhat/GPS)
   - The txt files are generated in the format GPS/LIDAR_2011_XX_XX_00XX
   - Use the scripts in the respective folders to run ICP and GPS scripts for desired dataset sequence 

3. KITTI Dataset files in the format 2011_XX_XX / 2011_XX_XX_drive_00XX_sync, Refer to the folder structure picture

4. TOML Configuration files which specify the following
   - KITTI dataset path 
   - Process and sensor noise parameters - The user can tune these for their specific sensor hardware and application


## Steps to run the sensor fusion

1. Specify the dataset path and edit desired noise params in the config file
2. Run the scripts fusion_X.py , refer to the commit for the model formulation details of different scripts
3. The script does the following in order
   - Read the TOML file and load the noise params into a nd array, KITTI dataset using PyKITTI
   - Load the respective ICP and GPS txt data files
   - Run the EKF - Predict and Correct functions in a loop for all the set of data points
   - Plot the results  



