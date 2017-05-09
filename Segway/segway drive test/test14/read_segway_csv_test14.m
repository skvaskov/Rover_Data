clear all;
close all;
%%%%%%%%%%%%%%%%%%% Read command velocities %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cmd_vel = csvread('_slash_segway_slash_cmd_vel.csv',1,0);
% Data format is: rosbagTimestamp,linear,x,y,z,angular,x,y,z
cmd_time = cmd_vel(:,1)/10^9;
cmd_lin = cmd_vel(:,3);
cmd_ang = cmd_vel(:,9)*1.5;%since my angular velocity limit is 1.5rad/s
start_col = find(cmd_ang,1);% Find the first non-zero point
start_time = cmd_time(start_col);
cmd_time = cmd_time - start_time;
figure
hold on
plot(cmd_time,cmd_ang,'--');
%%%%%%%%%%%%%%%%%%% Read feedback_wheel_odometry %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% fid = fopen('wheel_odometry.csv','r');
% dlmread() and csvread() are designed to only expect numeric values, 
% So we need to delete all the non-numerics values.(Finish this step in sublime or other editers)

% Original data format is:
% rosbagTimestamp,header,seq,stamp,secs,nsecs,frame_id,child_frame_id,pose,
% pose,position,x,y,z,orientation,x,y,z,w,covariance,twist,twist,linear,x,y,z,
% angular,x,y,z,covariance

% After deleting non numeric values, data format is:
% rosbagTimestamp,seq,secs,nsecs,(position)x,(position)y,(position)z,(orientation)x,(orientation)y,(orientation)z,(orientation)w,
% covariance(36numbers),(linear)x,(linear)y,(linear)z,(angular)x,(angular)y,(angular)z,covariance(36numbers)

feedback_wheel_odometry = csvread('feedback_wheel_odometry.csv');
feedback_wheel_time = feedback_wheel_odometry(:,3) + feedback_wheel_odometry(:,4)/10^9;
feedback_wheel_time = feedback_wheel_time - start_time;
feedback_wheel_lin = feedback_wheel_odometry(:,48);
feedback_wheel_ang = feedback_wheel_odometry(:,53);
plot(feedback_wheel_time,feedback_wheel_ang);

%%%%%%%%%%%%%%%%%%% Read odometry_local_filtered %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
odometry_local = csvread('odometry_local_filtered.csv');
odometry_local_time = odometry_local(:,3) + odometry_local(:,4)/10^9;
odometry_local_time = odometry_local_time - start_time;
odometry_local_lin = odometry_local(:,48);
odometry_local_ang = odometry_local(:,53);
plot(odometry_local_time,odometry_local_ang);
legend('command velocities','feedback wheel odometry','odometry local filtered','Location','best')
axis([-0.5 8 -0.5 2.5])
xlabel('Time (s)')
ylabel('Linear velocity (m/s)')
title({'Test 14, 0rad/s -> 0.75rad/s','acceleration limit: 10rad/s^2','angular limit: 1.5rad/s'})
