<h2>Steps to generate the simulation:</h2>

1. Run **car_simulator.py** and generate states of car by driving around using arrow keys.
2. The states which is the **ground truth** will be stored in form of csv file in Readings folder.
3. Use same map and run **distances(fd).py** to calculate the distances from LiDAR points to road edges.
4. These distances will later be used to calculate **likelihood** (in Bayes filter) of point present on road.
5. The distances will be stored as look up table in csv format in Distances folder.
6. Finally, run **simulation.py** with appropriate files and maps.
7. This will generate plots of distribution of pose of car (**belief** as said in Bayes filter) at each timestep.
8. **Compile** the plots to generate a video of simulation.
  (Compiling can done anyway, I have used **ffmpeg** as mentioned in simulation.py)
