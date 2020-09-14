# FYP-State-Estimation-SLIDER
Code for my final year project, a position estimator for the robot SLIDER at Imperial College London

The file estimator_SIM.cpp in slider_state/src implements the Kalman filter. 
ground_truth_receiver.cpp interfaces to the correct topics needed for the filter and tracks the error. 

Unfortunately the code cannot be run outside of the Robot Intelligence Lab as the Gazebo simulation of SLIDER is required.
