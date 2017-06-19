clear all
clc
close all
load('processed_circles_5_31.mat')
tarray=[2,3,5,6,7,8,9,10,11,13,14,16,17,18,19,20,21,22,23,24,25];

figure(1)
for i=1:length(tarray)
    subplot(2,2,1)
    plot(processeddata(tarray(i)).time,processeddata(tarray(i)).mocap.yawrate_smooth)
    hold on
    subplot(2,2,3)
    plot(processeddata(tarray(i)).time,processeddata(tarray(i)).mocap.longvelocity_smooth)
    hold on
    subplot(2,2,2)
    hold on
    plot(processeddata(tarray(i)).time,processeddata(tarray(i)).input.steering)
    subplot(2,2,4)
    hold on
    plot(processeddata(tarray(i)).time,processeddata(tarray(i)).input.throttle)

end

figure(1)
subplot(2,2,1)
ylabel('yaw rate (rad/s)')
xlabel('time (s)')
subplot(2,2,3)
ylabel('longitudinal velocity (m/s)')
xlabel('time (s)')
subplot(2,2,2)
ylabel('steering')
xlabel('time(s)')
subplot(2,2,4)
ylabel('throttle')
xlabel('time (s)')
