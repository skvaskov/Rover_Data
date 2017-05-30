%% 
%clear all then
%load workspace for trial with the following contents/format:
clear all
close all
clc
%enter trial name
trialname='straight1';
%enter file path to place where raw data is stored
filepath='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/';
load([filepath,trialname,'_workspace.mat'])
%make sv=1 if you want the step about saving stuff to run
sv=0;

%specifcy an endtime if you want to cut the data processing off at some
%point like if it goes out of the mocap range
endtime=4;
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

imusize=length(imutime);
encodersize=length(encodertime);
rcsize=length(rctime);
mocapsize=length(mocaptime);


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
headingbias=mean(headingmocap(1:10))-pi*mean(orientZ(1:10));
headingimu=pi*orientZ+headingbias;


%% smooth position, heading data, imu data
CYS=smooth(mocaptime,CPos(:,2),50,'sgolay',2);
CXS=smooth(mocaptime,CPos(:,1),50,'sgolay',2);
CPosS=[CXS,CYS];
headingmocapUS=smooth(mocaptime,unwrap(headingmocap));


figure
subplot(2,1,1)
plot(CPos(:,1),CPos(:,2))
hold on
plot(CPosS(:,1),CPosS(:,2))
hold off
subplot(2,1,2)
plot(mocaptime,headingmocap)
hold on
plot(mocaptime,headingmocapUS)
hold off


%smooth imu data
accellongimuCS=smooth(imutime,accellongimuC,25);
accellatimuCS=smooth(imutime,accellatimuC,25);
yawrateimuS=smooth(imutime,yawrateimu);
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
%% get and smooth other data from mocap results
close all
%derive speed (signed similar to long velocity), longitudal velocity, distance, lateral velocity from smooth position and heading data 
speedmocap=zeros(mocapsize,1);
distancemocap=zeros(mocapsize,1);
sideslipmocap=zeros(mocapsize,1);
yawratemocap=zeros(mocapsize,1);
v=zeros(2,mocapsize);

%initial point
 dS=CPosS(2,:)-CPosS(1,:);
 dt=mocaptime(2)-mocaptime(1);
 distancemocap(2)=norm(CPosS(2,:)-CPosS(1,:));
 OBA=[cos(headingmocapUS(1)) sin(headingmocapUS(1));-sin(headingmocapUS(1)) cos(headingmocapUS(1))];
 v(:,1)=OBA*dS'/dt;
 speedmocap(1)=sign(v(1,1))*norm(v(:,1));
 sideslipmocap(1)=atan2(v(2,1),v(1,1));
 yawratemocap(1)=(headingmocapUS(2)-headingmocapUS(1))/(mocaptime(2)-mocaptime(1));

%intermediatepoints
for i=2:mocapsize-1
    dS=CPosS(i+1,:)-CPosS(i-1,:);
    dt=mocaptime(i+1)-mocaptime(i-1);
    distancemocap(i+1)=distancemocap(i)+norm(CPosS(i+1,:)-CPosS(i,:));
    OBA=[cos(headingmocapUS(i)) sin(headingmocapUS(i));-sin(headingmocapUS(i)) cos(headingmocapUS(i))];
    v(:,i)=OBA*dS'/dt;
    speedmocap(i)=sign(v(1,i))*norm(v(:,i));
    sideslipmocap(i)=atan2(v(2,i),v(1,i));
    yawratemocap(i)=(headingmocapUS(i+1)-headingmocapUS(i-1))/(mocaptime(i+1)-mocaptime(i-1));
end

%final point
dS=CPosS(end,:)-CPosS(end-1,:);
dt=mocaptime(end)-mocaptime(end-1);
OBA=[cos(headingmocapUS(end)) sin(headingmocapUS(end));-sin(headingmocapUS(end)) cos(headingmocapUS(end))];
v(:,end)=OBA*dS'/dt;
speedmocap(end)=sign(v(1,end))*norm(v(:,end));
sideslipmocap(end)=atan2(v(2,end),v(1,end));
yawratemocap(end)=(headingmocapUS(end)-headingmocapUS(end-1))/(mocaptime(end)-mocaptime(end-1));

%smooth yaw rate and velocity, sidelsip
longvelocitymocap=v(1,:);
latvelocitymocap=v(2,:);
speedmocapS=smooth(mocaptime,speedmocap);
yawratemocapS=smooth(mocaptime,yawratemocap);
longvelocitymocapS=smooth(mocaptime,longvelocitymocap);
latvelocitymocapS=smooth(mocaptime,latvelocitymocap);
sideslipmocapS=smooth(sideslipmocap);
vS=[longvelocitymocapS';latvelocitymocapS'];

%derive acceleration from smooth velocity data 
accellongmocap=zeros(mocapsize,1);
accellatmocap=zeros(mocapsize,1);
sideslipmocapdt=zeros(mocapsize,1);

%initial point
dVdt=(vS(:,2)-vS(:,1))/(mocaptime(2)-mocaptime(1));
a=[dVdt;0]+cross([0;0;yawratemocapS(1)],[vS(:,1);0]);
accellongmocap(1)=a(1);
accellatmocap(1)=a(2);
sideslipmocapdt(1)=(sideslipmocapS(2)-sideslipmocapS(1))/(mocaptime(2)-mocaptime(1));
%intermediate points    
for i=2:mocapsize-1
    dVdt=(vS(:,i+1)-vS(:,i-1))/(mocaptime(i+1)-mocaptime(i-1));
    a=[dVdt;0]+cross([0;0;yawratemocapS(i)],[vS(:,i);0]);
    accellongmocap(i)=a(1);
    accellatmocap(i)=a(2);
    sideslipmocapdt(i)=(sideslipmocapS(i+1)-sideslipmocapS(i-1))/(mocaptime(i+1)-mocaptime(i-1));
end
%end points
dVdt=(vS(:,end)-vS(:,end-1))/(mocaptime(end)-mocaptime(end-1));
a=[dVdt;0]+cross([0;0;yawratemocapS(end)],[vS(:,end);0]);
accellongmocap(end)=a(1);
accellatmocap(end)=a(2);
sideslipmocapdt(end)=(sideslipmocapS(end)-sideslipmocapS(end-1))/(mocaptime(end)-mocaptime(end-1));

%smooth data
accellongmocapS=smooth(mocaptime,accellongmocap)';
accellatmocapS=smooth(mocaptime,accellatmocap)';
sideslipmocapdtS=smooth(mocaptime,sideslipmocapdt)';

%% shift time vectors prior to interpolation
close all

%shift and scale imu and mocap time to be zero at approriate points
shiftedimutime=imutime-imutime(imustart);
timedelay=imutime(imustart)-rctime(rcstart);
shiftedrctime=rctime-rctime(rcstart);
encoderbias=-.11;
shiftedencodertime=encodertime-imutime(imustart)+encoderbias;
mocapbias=0;
shiftedmocaptime=(mocaptime-mocaptime(mocapstart))+mocapbias;

%get vector for time (all interpolated data will correspond to this time
%cut time vector off at nearest index
timeidx1=find(shiftedimutime>shiftedencodertime(end),1)-1;
timeidx2=find(shiftedimutime>shiftedmocaptime(end),1)-1;
if timeidx1>timeidx2
    timeidx=timeidx2;
else
    timeidx=timeidx1;
end
if exist('endtime','var')
    timeidx=find(shiftedimutime>endtime,1)-1;
end
    
time=shiftedimutime(imustart:timeidx);
datasize=length(time);


%% interpolate
%trim the data from the imu since it is on this timestep
accellongimuCI=accellongimuC(imustart:timeidx);
accellatimuCI=accellatimuC(imustart:timeidx);
yawrateimuI=angvZ(imustart:timeidx);
accellongimuCSI=accellongimuCS(imustart:timeidx);
accellatimuCSI=accellatimuCS(imustart:timeidx);
yawrateimuSI=yawrateimuS(imustart:timeidx);
headingimuI=headingimu(imustart:timeidx);
headingimuUSI=headingimuUS(imustart:timeidx);
%interpolate mocapdata
posmocapI=interp1(shiftedmocaptime,CPos,time);
posmocapSI=interp1(shiftedmocaptime,CPosS,time);
headingmocapI=interp1(shiftedmocaptime,headingmocap,time);
headingmocapUSI=interp1(shiftedmocaptime,headingmocapUS,time);

speedmocapI=interp1(shiftedmocaptime,speedmocap,time);
speedmocapSI=interp1(shiftedmocaptime,speedmocapS,time);
distancemocapI=interp1(shiftedmocaptime,distancemocap,time);
longvelocitymocapI=interp1(shiftedmocaptime,longvelocitymocap,time);
longvelocitymocapSI=interp1(shiftedmocaptime,longvelocitymocapS,time);
latvelocitymocapI=interp1(shiftedmocaptime,latvelocitymocap,time);
latvelocitymocapSI=interp1(shiftedmocaptime,latvelocitymocapS,time);
sideslipmocapI=interp1(shiftedmocaptime,sideslipmocap,time);
sideslipmocapSI=interp1(shiftedmocaptime,sideslipmocapS,time);
yawratemocapI=interp1(shiftedmocaptime,yawratemocap,time);
yawratemocapSI=interp1(shiftedmocaptime,yawratemocapS,time);

accellatmocapI=interp1(shiftedmocaptime,accellatmocap,time);
accellatmocapSI=interp1(shiftedmocaptime,accellatmocapS,time);
accellongmocapI=interp1(shiftedmocaptime,accellongmocap,time);
accellongmocapSI=interp1(shiftedmocaptime,accellongmocapS,time);
sideslipmocapdtI=interp1(shiftedmocaptime,sideslipmocapdt,time);
sideslipmocapdtSI=interp1(shiftedmocaptime,sideslipmocapdtS,time);


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
subplot(4,2,1)
hold on
plot(posmocapSI(1,1),posmocapSI(1,2),'b*')
plot(posmocapSI(end,1),posmocapSI(end,2),'r*')
plot(posmocapSI(:,1),posmocapSI(:,2),'g')
legend('Start','Finish')
ylabel('Y position')
xlabel('X position')
hold off

subplot(4,2,2)
hold on
plot(time,leftencoderI)
plot(time,leftencoderIS)
plot(time,rightencoderI)
plot(time,rightencoderIS)
ylabel('Encoderdata')
xlabel('Time')
legend('Left','Left Smooth', 'Right','Right Smooth')

subplot(4,2,3)
plot(time,headingmocapI,'b')
hold on
plot(time,headingmocapUSI,'r')
%plot(time,headingimuI,'g')
%plot(time,headingimuUSI,'m')
%plot(time,sideslipmocapI,'g')
%plot(time,sideslipmocapSI,'m')
legend('calculated','unwrapped & smoothed')
ylabel('heading')
xlabel('Time')
hold off

subplot(4,2,4)
plot(time,speedmocapI,'k')
hold on
plot(time,speedmocapSI,'c')
plot(time,longvelocitymocapI,'b')
plot(time,longvelocitymocapSI,'r')
plot(time,latvelocitymocapI,'g')
plot(time,latvelocitymocapSI,'m')
ylabel('Veocity')
xlabel('Time')
legend('speed','speed smooth','long v', 'long v smooth','lat v', 'lat v smooth')
hold off

subplot(4,2,5)
plot(time,accellongmocapI,'b')
hold on
plot(time,accellongmocapSI,'r')
plot(time,accellatmocapI,'g')
plot(time,accellatmocapSI,'m')
legend('long','long smooth', 'lat', 'lat smooth')
ylabel('Acceleration (Mocap)')
xlabel('Time')

hold off

subplot(4,2,6)
plot(time,accellongimuCI,'b')
hold on
plot(time,accellongimuCSI,'r')
plot(time,accellatimuCI,'g')
plot(time,accellatimuCSI,'m')
legend('long','long smooth', 'lat', 'lat smooth')
ylabel('Acceleration (imu)')
xlabel('Time')
hold off

subplot(4,2,7)
%plot(time,sideslipmocapdtI,'k')
hold on
%plot(time,sideslipmocapdtSI,'c')
plot(time,yawratemocapI,'b')
plot(time,yawratemocapSI,'r')
plot(time,yawrateimuI,'g')
plot(time,yawrateimuSI,'m')
xlabel('Time')
ylabel('Yaw Rate ')
legend('mocap', 'mocap smooth', 'imu', 'imu smooth')


hold off

subplot(4,2,8)
plot(time,throttlercI,'b')
hold on
%plot(time,throttlercIS,'r')
plot(time,steeringrcI,'g')
%plot(time,steeringrcIS,'m')
ylabel('Inputs')
xlabel('Time')
legend('throttle', 'steering')
hold off

figure
hold on
plot(time,yawratemocapI,'b')
plot(time,yawratemocapSI,'r')
plot(time,yawrateimuI,'g')
plot(time,yawrateimuSI,'m')
xlabel('Yaw Rate/SideSlip')
legend('Mocap','Mocap Smooth','Imu','Imu Smooth')
% figure(3)
% plot(time,headingmocapI,'b')
% hold on
% plot(time,headingmocapUSI,'r')
% plot(time,headingimuI,'g')
% plot(time,headingimuUSI,'m')
figure
hold on
plot(time,leftencoderI)
plot(time,leftencoderIS)
plot(time,rightencoderI)
plot(time,rightencoderIS)
plot(time,speedmocapSI,'c')
plot(time,longvelocitymocapSI,'r')
legend('Left','Left Smooth','Right','Right Smooth','Speed Mocap Smooth','Long V Mocap Smooth')
%% 
if sv
%Define variable containing data you want to use for sysID and clear other
%variables see text file for order
processed_straight1=[time,posmocapSI,distancemocapI,...
    speedmocapI,speedmocapSI,longvelocitymocapI,longvelocitymocapSI,latvelocitymocapI,latvelocitymocapSI,...
    accellongimuCI,accellongimuCSI,accellatimuCI,accellatimuCSI,...
    accellongmocapI,accellongmocapSI,accellatmocapI,accellatmocapSI,...
    headingmocapI,headingmocapUSI,sideslipmocapI,sideslipmocapSI,...
    yawrateimuI,yawrateimuSI,yawratemocapI,yawratemocapSI,sideslipmocapdtI,sideslipmocapdtSI,...
    leftencoderI,leftencoderIS,rightencoderI,rightencoderIS,...
    throttlercI,throttlercIS,steeringrcI,steeringrcIS]';

clearvars -except processed*
save('may25_tests.mat','processed*','-append')
end
%% 

