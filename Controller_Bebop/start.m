%%%%%%%%%%%%%%%%%%%%%%%%
%  cleaning workspace
clear;
close all;
clc;
% A_v=0.8;
% A_p=0.8;
% delta_p = 0.2;
k_l=5;
k=1.0;
eps=0.001;
r_M=2;
r_a=3;
r_s=0.001;
r_o=0.5;
Is_sim = 0; % simulation gazebo
sampleTime = 0.01; 
k_p_xy = 0.6;
k_d_xy = 0.9;
% k_i_xy = 0.005;
k_p_z = 0.6;
k_d_z = 0.3;
if Is_sim == 1
    k_p_yaw = -0.5;
    k_sign = 1;
else
    k_p_yaw = 0.5;
    k_sign = -1;
end
limit_cmd_xy = 1;
%%%%%%%%%%%%%%%%%%%%%%%%
try
% 	setenv('ROS_MASTER_URI','http://192.168.0.2:11311') % the IP of server 
% 	setenv('ROS_IP','192.168.0.3') % the IP of client
% 	rosinit('http://192.168.0.2:11311')
rosinit;
end
uav_cmd;
% SingleARDroneCtrl; 
% waypoints1=getWaypoints(0,A_p,A_v);
waypoints = [1,1,1.2,0,0,0,0]';
parameters=[k_l,k,eps,r_M,r_a,r_s,r_o]';
% waypoints2=getWaypoints(-pi,A_p,A_v);
r2017bSingleARDroneCtrl