%find calibration matrices for imu data
clear all
close all
clc
%enter trial name
trialname='circles22_5_31_workspace.mat';
%enter file path to place where raw data is stored
filepath='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/';
load([filepath,trialname])

%%
shiftedimutime=imutime-imutime(imustart);
shiftedmocaptime=mocaptime-mocaptime(mocapstart);
timeidx=find(shiftedimutime>shiftedmocaptime(end),1)-1;
if isempty(timeidx)
    timeidx=length(shiftedimutime);
end
time=shiftedimutime(imustart:timeidx);

orientationimuI=interp1(shiftedimutime,unwrap(orientationimu,[],2)',time)';
orientationmocapI=interp1(shiftedmocaptime,unwrap(orientationmocap_conc,[],2)',time)';
[regParams,Bfit,ErrorStats]=absor(orientationimuI,orientationmocapI);
orientimu_cal=regParams.R*orientationimuI+regParams.t;
figure
subplot(3,1,1)
plot(time,orientationmocapI(3,:))
hold on
plot(time,orientimu_cal(3,:))
ylabel('heading')
legend('mocap','corrected imu')
subplot(3,1,2)
plot(orientationmocapI(1,:))
hold on
plot(orientimu_cal(1,:))
ylabel('roll')
legend('mocap','corrected imu')
subplot(3,1,3)
plot(orientationmocapI(2,:))
hold on
plot(orientimu_cal(2,:))
ylabel('pitch')
legend('mocap','corrected imu')