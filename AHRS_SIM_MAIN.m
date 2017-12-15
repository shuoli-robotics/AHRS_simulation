clear 
clear global
close all
clc

global GT IMU AHRS

generate_ground_truth_maneuver(2,100);
generate_IMU_measurement()
%complementary_filter();
EKF_6_STATES();
temp = 1;