clear 
clear global
close all
clc

global GT IMU AHRS

generate_ground_truth_maneuver(1,50);
generate_IMU_measurement()
complementary_filter();
temp = 1;