%% 
%clear all then
%load workspace for trial with the following contents/format:
clear all
close all
clc

%%    
%enter trial number
num=16;
%make sv=1 if you want the step about saving stuff to run
sv=1;
trialname=['straight',num2str(num),'_5_31'];
groupname='processed_straight_5_31.mat';
filepathdata='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/';
filepathgroup='~/Documents/MATLAB/Rover_Data/sysid_summer2017/';
%number in data structure

%enter file path to place where raw data is stored
load([filepathdata,trialname,'_workspace.mat'])




%specifcy an endtime if you want to cut the data processing off at some
%point like if it goes out of the mocap range or got picked up
endtime=9;
%all imported data will be in columns

%RC imu
%time: imutime
%angular velocity: angvX, angvY, angvZ
%linear accelaration: accX,accY,accZ
%orientation: orientW,orientX,orientY,orientZ
%index before imu movement: imustart

%Encoders
%time: encodertime
%left/right encoder data: leftencoder,rightencoder
%index before encoder movement:encoderstart

%Input
%time: rctime
%steering channel: Input1
%Throttle channel: Input2
%index before first input: rcstart

%Mocap
%time: mocaptime
%center (X,Y) positon: CPos
%heading: headingmocap
%Position (ntimesteps x markers([FC,FL,SL,RL,RC,RR,SR,FR])): X,Y,Z
%index before mocap movement:mocapstart

%specify start index of mocap and imu data (point before first motion):
%mocapstart, imustart


%this version lines mocap time up with the time from the imu
%it also lines the time from the input recordings up with the imu
%and I will manually line up the angular velocities. We shift the timestamp
%for the encoder by the same as the imu

%%
%this part creates a new data structure if needed
if exist([filepathgroup,groupname],'file')
load([filepathgroup,groupname])
    if exist('processeddata','var')==0
        processeddata=struct('name',{},'time',{},'mocap',{},'imu',{},'encoder',{},'input',{});
    end
else
    save([filepathgroup,groupname],'groupname')
end
%%
processeddata(num).imu=struct('longaccel',{},'longaccel_smooth',{},'lataccel',{},'lataccel_smooth',{},...
         'yawrate',{},'yawrate_smooth',{},'heading',{},'heading_unwrapped_smooth',{});
 processeddata(num).encoder=struct('left',{},'left_smooth',{},'right',{},'right_smooth',{});
     processeddata(num).input=struct('throttle',{},'throttle_smooth',{},'steering',{},'steering_smooth',{});
processeddata(num).name=trialname;
imusize=length(imutime);
if exist('encodersize','var')
encodersize=length(encodertime);
end
rcsize=length(rctime);

%% define rotation matrices and scaling
%to calibrate Accelerometer data
ROT2= [0.999979208333881 0 -0.006448480436904;...
       0 1 0;... 
   0.006448480436904 0 0.999979208333881];
ROT3=[1 0 0;...
    0 0.999826874360574 0.018607023038773;...
    0 -0.018607023038773 0.999826874360574];
zavg=0.964832691296737;
g=9.80665;

%calibration matrix for acceleration from model
A=[9.826129556490821315e+00 1.105448087359244859e-02 -4.505705534461542511e-02;...
1.105448087359184837e-02 9.849205825189187635e+00 3.365730156842951931e-02;...
-4.505705534461620226e-02 3.365730156842847848e-02 9.888877421200394480e+00];
b=[6.484171140149968399e-03;7.233596302086491014e-02;-2.683383240809078529e-01];

%% calibrate acceleration and other imu data
araw=[accX';accY';accZ'];

acal=g/zavg*ROT3'*ROT2'*(A\(araw-b));

accellongimu=acal(1,:)';
accellatimu=acal(2,:)';
%correct acceraltion to 0
accellongimuC=accellongimu-mean(accellongimu(1:imustart));
accellatimuC=accellatimu-mean(accellatimu(1:imustart));
yawrateimu=angvZ;

%heading data
headingbias=0;
headingimu=pi*orientZ+headingbias;


%% smooth position, heading data, imu data


%smooth imu data
accellongimuCS=smooth(imutime,accellongimuC,25);
accellatimuCS=smooth(imutime,accellatimuC,25);
yawrateimuS=smooth(imutime,yawrateimu,50);
headingimuUS=smooth(imutime,unwrap(headingimu));

figure
subplot(2,2,1)
plot(imutime,accellongimu)
hold on
plot(imutime,accellongimuCS)
subplot(2,2,2)
plot(imutime,accellatimuC)
hold on
plot(imutime,accellatimuCS)
subplot(2,2,3)
plot(imutime,yawrateimu)
hold on
plot(imutime,yawrateimuS)
subplot(2,2,4)
plot(imutime,headingimu)
hold on
plot(imutime,headingimuUS)


%% shift time vectors prior to interpolation
close all

%shift and scale imu and mocap time to be zero at approriate points
shiftedimutime=imutime-imutime(imustart);
timedelay=imutime(imustart)-rctime(rcstart);
shiftedrctime=rctime-rctime(rcstart);
encoderbias=-.11;
shiftedencodertime=encodertime-imutime(imustart)+encoderbias;

%get vector for time (all interpolated data will correspond to this time
%cut time vector off at nearest index
timeidx=find(shiftedimutime>shiftedencodertime(end),1)-1;

if exist('endtime','var')
    timeidx=find(shiftedimutime>endtime,1)-1;
end
if isempty(timeidx)
    timeidx=length(shiftedimutime);
end
    
time=shiftedimutime(imustart:timeidx);
datasize=length(time);


%% interpolate
%trim the data from the imu since it is on this timestep
accellongimuCI=accellongimuC(imustart:timeidx);
accellatimuCI=accellatimuC(imustart:timeidx);
yawrateimuI=yawrateimu(imustart:timeidx);
accellongimuCSI=accellongimuCS(imustart:timeidx);
accellatimuCSI=accellatimuCS(imustart:timeidx);
yawrateimuSI=yawrateimuS(imustart:timeidx);
headingimuI=headingimu(imustart:timeidx);
headingimuUSI=headingimuUS(imustart:timeidx);

%interpolate input and encoder
steeringrcI=interp1(shiftedrctime,steeringrc,time);
throttlercI=interp1(shiftedrctime,throttlerc,time);
leftencoderI=interp1(shiftedencodertime,leftencoder,time);
rightencoderI=interp1(shiftedencodertime,rightencoder,time);

%% final smoothing of encoder, inputs
steeringrcIS=smooth(time,steeringrcI);
throttlercIS=smooth(time,throttlercI);
leftencoderIS=smooth(time,leftencoderI,25,'sgolay',3);
rightencoderIS=smooth(time,rightencoderI,25,'sgolay',3);

%% 
%plot data
%all data should be plotted against "time", which is the timesteps aligned
%with the input channels
%variableformat=datasourceXY
%data is pos,heading,acceltan,velocity,steering,or throttle
%source is mocap,imu, or rc
%X is I if the variable has been interpolated to line up "time", or nothing otherwise
%Y is S if the data has a smoothing function applied to it, or C if its
%corrected somehow

figure


subplot(2,2,1)
hold on
plot(time,leftencoderI)
plot(time,leftencoderIS)
plot(time,rightencoderI)
plot(time,rightencoderIS)
ylabel('Encoderdata')
xlabel('Time')
legend('Left','Left Smooth', 'Right','Right Smooth')

subplot(2,2,2)
plot(time,headingimuI,'b')
hold on
plot(time,headingimuUSI,'r')
legend('measured','unwrapped & smoothed')
ylabel('heading (imu)')
xlabel('Time')
hold off


subplot(2,2,3)
plot(time,accellongimuCI,'b')
hold on
plot(time,accellongimuCSI,'r')
plot(time,accellatimuCI,'g')
plot(time,accellatimuCSI,'m')
legend('long','long smooth', 'lat', 'lat smooth')
ylabel('Acceleration (imu)')
xlabel('Time')
hold off

subplot(2,2,4)
plot(time,yawrateimuI,'g')
hold on
plot(time,yawrateimuSI,'m')
xlabel('Time')
ylabel('Yaw Rate ')
legend('imu', 'imu smooth')



figure
hold on
plot(time,leftencoderI)
plot(time,leftencoderIS)
plot(time,rightencoderI)
plot(time,rightencoderIS)
plot(time,accellongimuCSI,'r')
legend('Left','Left Smooth','Right','Right Smooth','Smooth Accel Mocap')
%% 
if sv
    processeddata(num).time=time;
processeddata(num).imu(1).longaccel=accellongimuCI;
 processeddata(num).imu(1).longaccel_smooth=accellongimuCSI;
 processeddata(num).imu(1).lataccel=accellatimuCI;
 processeddata(num).imu(1).lataccel_smooth=accellatimuCSI;
 processeddata(num).imu(1).yawrate=yawrateimuI;
 processeddata(num).imu(1).yawrate_smooth=yawrateimuSI;
 processeddata(num).imu(1).heading=headingimuI;
 processeddata(num).imu(1).heading_unwrapped_smooth=headingimuUSI;
if exist('leftencoder','var')
processeddata(num).encoder(1).left=leftencoderI;
processeddata(num).encoder(1).left_smooth=leftencoderIS;
end
if exist('rightencoder','var')
processeddata(num).encoder(1).right=rightencoderI;
processeddata(num).encoder(1).right_smooth=rightencoderIS;
end
processeddata(num).input(1).throttle=throttlercI;
processeddata(num).input(1).throttle_smooth=throttlercIS;
processeddata(num).input(1).steering=steeringrcI;
processeddata(num).input(1).steering_smooth=steeringrcIS;

clearvars -except processeddata* groupname filepathgroup
save([filepathgroup,groupname])
end
%% 

