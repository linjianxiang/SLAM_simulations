# MATLAB SLAM simulations --EKF,UKF,GRAPH SLAM
Simulates 2D EKF,UKF,and graph-based SLAM.

The folder "slam_code" and "slam_code_2wheel_model" uses different motion model for simulation

Setups:
The simulation generated a 2D map with a number of landmarks. 
The 2D Laser scan is simulated as distance+noise
The simulated robot run in a circuler motion

SLAM:
EKF, and UKF SLAM are run for landmark mapping and robot localization. 
After that, 
Graph-based optimization run on the result from EKF and UKF SLAM. 
For the Graph SLAM, constrains are added between every step and loop-closure constrains are randomly generated


run kf\_tb.m to start the program
