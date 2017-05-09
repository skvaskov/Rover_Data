clear all;
close all;
%%%%%%%%%%%%%%%%%%% Read command velocities %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cmd_vel = csvread('_slash_segway_slash_cmd_vel.csv',1,0);
% Data format is: rosbagTimestamp,linear,x,y,z,angular,x,y,z
cmd_time = cmd_vel(:,1)/10^9;
cmd_lin = cmd_vel(:,3);
start_col = find(cmd_lin,1);% Find the first non-zero point
start_time = cmd_time(start_col);
cmd_time = cmd_time - start_time;
fig=figure(1)
hold on
plot(cmd_time,cmd_lin,'--');
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
plot(feedback_wheel_time,feedback_wheel_lin);

%%%%%%%%%%%%%%%%%%% Read odometry_local_filtered %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
odometry_local = csvread('odometry_local_filtered.csv');
odometry_local_time = odometry_local(:,3) + odometry_local(:,4)/10^9;
odometry_local_time = odometry_local_time - start_time;
odometry_local_lin = odometry_local(:,48);
plot(odometry_local_time,odometry_local_lin);
legend('command velocities','feedback wheel odometry','odometry local filtered','Location','best')
axis([-1 6 -0.5 1.5])
xlabel('Time (s)')
ylabel('Linear velocity (m/s)')
title('Test 10, 0.5m/s -> 1m/s, 7m/s^2')
