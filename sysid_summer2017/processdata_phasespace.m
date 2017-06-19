%% 
%clear all then
%load workspace for trial with the following contents/format:
clear all
close all
clc
speed=[1670,1680,1690];
trialnames={};
%for is=1:3
    for it=1:1
      trialnames=[trialnames;'pwm',num2str(it),'A'];
    end
    for it=8:23
      trialnames=[trialnames;'pwm',num2str(it),'A'];
    end
    for it=2:7
      trialnames=[trialnames;'pwm',num2str(it),'B'];
    end
%end

clearvars speed is it is1 is2
%%    
groupname='motor_pwm.mat';
filepathgroup='~/Documents/MATLAB/Rover_Data/sysid_summer2017/';

%enter trial number
for num=9:9
    

%make sv=1 if you want the step about saving stuff to run
sv=0;

%
trialname=trialnames{num};
filepathdata='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/workspaces/';

%number in data structure

%enter file path to place where raw data is stored
load([filepathdata,trialname,'_workspace.mat'])
if rcstart==0
    rcstart=1;
end
%specifcy an endtime if you want to cut the data processing off at some
%point like if it goes out of the mocap range
%endtime=16.9;
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
processeddata(num).mocap=struct('pos',{},'distance',{},'speed',{},'speed_smooth',{},...
        'longvelocity',{},'longvelocity_smooth',{},'latvelocity',{},'latvelocity_smooth',{},...
        'longaccel',{},'longaccel_smooth',{},'lataccel',{},'lataccel_smooth',{},...
         'heading',{},'heading_unwrapped',{},'heading_unwrapped_smooth',{},'bodyslip',{},'bodyslip_smooth',{},...
         'yawrate',{},'yawrate_smooth',{},'bodyslip_dt',{},'bodyslip_dt_smooth',{});
 processeddata(num).imu=struct('longaccel',{},'longaccel_smooth',{},'lataccel',{},'lataccel_smooth',{},...
         'yawrate',{},'yawrate_smooth',{},'heading',{},'heading_unwrapped',{},'heading_unwrapped_smooth',{});
 processeddata(num).encoder=struct('left',{},'left_smooth',{},'right',{},'right_smooth',{});
     processeddata(num).input=struct('throttle',{},'throttle_smooth',{},'steering',{},'steering_smooth',{});
processeddata(num).name=trialname;
imusize=length(imutime);
if exist('encodersize','var')
encodersize=length(encodertime);
end
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
headingimu=orientationimu_cal(3,:);


%% smooth position, heading data, imu data
CYS=smooth(mocaptime,CPos_step(2,:));
CXS=smooth(mocaptime,CPos_step(1,:));
CZS=smooth(mocaptime,CPos_step(3,:));
CPosS=[CXS';CYS';CZS'];
headingmocap=orientationmocap_conc(3,:);
rollmocap=orientationmocap_conc(2,:);
pitchmocap=orientationmocap_conc(1,:);

headingmocapU=unwrap(headingmocap);
hrd=diff(headingmocapU);
idxs=find(abs(hrd)>.25);
idxs=unique([idxs-1;idxs;idxs+1]);
headingmocapU(idxs)=[];
mocaptimehr=mocaptime;
mocaptimehr(idxs)=[];
headingmocapU=interp1(mocaptimehr,headingmocapU,mocaptime);
headingmocapUS=smooth(mocaptime,headingmocapU);
pitchmocapS=smooth(mocaptime,pitchmocap);
rollmocapS=smooth(mocaptime,rollmocap);
% 
% figure
% subplot(2,1,1)
% plot(CPos_step(1,:),CPos_step(2,:))
% hold on
% plot(CPos_step(1,1),CPos_step(2,1),'r*')
% plot(CPosS(1,:),CPosS(2,:))
% hold off
% subplot(2,1,2)
% plot(mocaptime,headingmocap)
% hold on
% plot(mocaptime,headingmocapUS)
% hold off


%smooth imu data
accellongimuCS=smooth(imutime,accellongimuC);
accellatimuCS=smooth(imutime,accellatimuC);
yawrateimuS=smooth(imutime,yawrateimu);
headingimuU=unwrap(headingimu);
headingimuUS=smooth(imutime,headingimuU);


% figure
% subplot(2,2,1)
% plot(imutime,accellongimu)
% hold on
% plot(imutime,accellongimuCS)
% subplot(2,2,2)
% plot(imutime,accellatimuC)
% hold on
% plot(imutime,accellatimuCS)
% subplot(2,2,3)
% plot(imutime,yawrateimu)
% hold on
% plot(imutime,yawrateimuS)
% subplot(2,2,4)
% plot(imutime,headingimu)
% hold on
% plot(imutime,headingimuUS)
%% get and smooth other data from mocap results
%close all
%derive speed (signed similar to long velocity), longitudal velocity, distance, lateral velocity from smooth position and heading data 
distancemocap=zeros(mocapsize,1);
for i=2:mocapsize
    distancemocap(i)=norm(CPosS(:,i)-CPosS(:,i-1))+distancemocap(i-1);
end
I=repmat(eye(3),1,1,mocapsize);
ov=get_velocity([rollmocapS';pitchmocapS';headingmocapUS'],I,mocaptime);
pitchratemocap=ov(2,:);
rollratemocap=ov(1,:);
yawratemocap=ov(3,:);
v=get_velocity(CPosS,Rot_conc,mocaptime);
speedmocap=sign(v(1,:)).*sqrt(sum(v(1,:).^2+v(2,:).^2,1));
sideslipmocap=atan2(v(2,:),v(1,:));
%smooth yaw rate and velocity, sidelsip
longvelocitymocap=v(1,:);
latvelocitymocap=v(2,:);
speedmocapS=smooth(mocaptime,speedmocap);
yawratemocapS=smooth(mocaptime,yawratemocap);
pitchratemocapS=smooth(mocaptime,yawratemocap);
rollratemocapS=smooth(mocaptime,yawratemocap);
longvelocitymocapS=smooth(mocaptime,longvelocitymocap);
latvelocitymocapS=smooth(mocaptime,latvelocitymocap);
sideslipmocapS=smooth(mocaptime,sideslipmocap);
vS=[longvelocitymocapS';latvelocitymocapS';smooth(mocaptime,v(3,:))'];
angVS=[rollratemocapS';pitchratemocapS';yawratemocapS'];
%derive acceleration from smooth velocity data 

sideslipmocapdt=get_velocity(sideslipmocap,ones(1,1,mocapsize),mocaptime);


accel=get_acceleration(vS,angVS,mocaptime);
accellongmocap=accel(1,:);
accellatmocap=accel(2,:);
%smooth data
accellongmocapS=smooth(mocaptime,accellongmocap);
accellatmocapS=smooth(mocaptime,accellatmocap);
sideslipmocapdtS=smooth(mocaptime,sideslipmocapdt);

%% shift time vectors prior to interpolation
%close all

%shift and scale imu and mocap time to be zero at approriate points
shiftedimutime=imutime-imutime(imustart);
timedelay=imutime(imustart)-rctime(rcstart);
shiftedrctime=rctime-rctime(rcstart);
shiftedmocaptime=(mocaptime-mocaptime(mocapstart));
shiftedencodertime=encodertime-imutime(imustart)-.11;

while sv~=1
if exist('leftencoder','var')
encoderbias=input(['Trial ',num2str(num),', enter encoder bias: ']);
shiftedencodertime=shiftedencodertime+encoderbias;
end


mocapbias=input(['Trial ',num2str(num),', enter mocap bias: ']);
shiftedmocaptime=shiftedmocaptime+mocapbias;

%get vector for time (all interpolated data will correspond to this time
%cut time vector off at nearest index
if exist('encodertime','var')
timeidx1=find(shiftedimutime>shiftedencodertime(end),1)-1;
timeidx2=find(shiftedimutime>shiftedmocaptime(end),1)-1;
if timeidx1>timeidx2
    timeidx=timeidx2;
else
    timeidx=timeidx1;
end
else
    timeidx=find(shiftedimutime>shiftedmocaptime(end),1)-1;
end

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
yawrateimuI=angvZ(imustart:timeidx);
accellongimuCSI=accellongimuCS(imustart:timeidx);
accellatimuCSI=accellatimuCS(imustart:timeidx);
yawrateimuSI=yawrateimuS(imustart:timeidx);
headingimuI=headingimu(imustart:timeidx);
headingimuUI=headingimu(imustart:timeidx);
headingimuUSI=headingimuUS(imustart:timeidx);
%interpolate mocapdata
posmocapI=interp1(shiftedmocaptime,CPos_step',time);
posmocapSI=interp1(shiftedmocaptime,CPosS',time);
headingmocapI=interp1(shiftedmocaptime,headingmocap,time);
headingmocapUSI=interp1(shiftedmocaptime,headingmocapUS,time);
headingmocapUI=interp1(shiftedmocaptime,headingmocapU,time);
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
steeringrcIS=smooth(time,steeringrcI);
throttlercI=interp1(shiftedrctime,throttlerc,time);
throttlercIS=smooth(time,throttlercI);

if exist('leftencoder','var')
leftencoderI=interp1(shiftedencodertime,leftencoder,time);
leftencoderIS=smooth(time,leftencoderI);
end
if exist('rightencoder','var')
rightencoderI=interp1(shiftedencodertime,rightencoder,time);
rightencoderIS=smooth(time,rightencoderI);
end

%% 
%plot data
%all data should be plotted against "time", which is the timesteps aligned
%with the input channels
%variableformat=datasourceXY
%data is pos,heading,acceltan,velocity,steering,or throttle
%source is mocap,imu, or rc
%X is I if the variable has been interpolated to line up "time", or nothing otherwise
%Y is S if the data has a smoothing function applied to it, or C if its
% %corrected somehow
% 
% figure(1)
% subplot(4,2,1)
% hold on
% plot(posmocapSI(1,1),posmocapSI(1,2),'b*')
% plot(posmocapSI(end,1),posmocapSI(end,2),'r*')
% plot(posmocapSI(:,1),posmocapSI(:,2),'g')
% legend('Start','Finish')
% ylabel('Y position')
% xlabel('X position')
% hold off
% if exist('rightencoder','var')
% subplot(4,2,2)
% hold on
% plot(time,leftencoderI)
% plot(time,leftencoderIS)
% plot(time,rightencoderI)
% plot(time,rightencoderIS)
% hold off
% ylabel('Encoderdata')
% xlabel('Time')
% legend('Left','Left Smooth', 'Right','Right Smooth')
% end
% 
% subplot(4,2,3)
% plot(time,headingmocapI,'b')
% hold on
% plot(time,headingmocapUSI,'r')
% plot(time,headingimuI,'g')
% plot(time,headingimuUSI,'m')
% %plot(time,sideslipmocapI,'g')
% %plot(time,sideslipmocapSI,'m')
% legend('calculated','unwrapped & smoothed')
% ylabel('heading')
% xlabel('Time')
% hold off
% 
% subplot(4,2,4)
% plot(time,speedmocapI,'k')
% hold on
% plot(time,speedmocapSI,'c')
% plot(time,longvelocitymocapI,'b')
% plot(time,longvelocitymocapSI,'r')
% plot(time,latvelocitymocapI,'g')
% plot(time,latvelocitymocapSI,'m')
% ylabel('Veocity')
% xlabel('Time')
% legend('speed','speed smooth','long v', 'long v smooth','lat v', 'lat v smooth')
% hold off
% 
% subplot(4,2,5)
% plot(time,accellongmocapI,'b')
% hold on
% plot(time,accellongmocapSI,'r')
% plot(time,accellatmocapI,'g')
% plot(time,accellatmocapSI,'m')
% legend('long','long smooth', 'lat', 'lat smooth')
% ylabel('Acceleration (Mocap)')
% xlabel('Time')
% 
% hold off
% 
% subplot(4,2,6)
% plot(time,accellongimuCI,'b')
% hold on
% plot(time,accellongimuCSI,'r')
% plot(time,accellatimuCI,'g')
% plot(time,accellatimuCSI,'m')
% legend('long','long smooth', 'lat', 'lat smooth')
% ylabel('Acceleration (imu)')
% xlabel('Time')
% hold off
% 
% subplot(4,2,7)
% %plot(time,sideslipmocapdtI,'k')
% hold on
% %plot(time,sideslipmocapdtSI,'c')
% plot(time,yawratemocapI,'b')
% plot(time,yawratemocapSI,'r')
% plot(time,yawrateimuI,'g')
% plot(time,yawrateimuSI,'m')
% hold off
% xlabel('Time')
% ylabel('Yaw Rate ')
% legend('mocap', 'mocap smooth', 'imu', 'imu smooth')

% 
% subplot(4,2,8)
% plot(time,throttlercI,'b')
% hold on
% %plot(time,throttlercIS,'r')
% plot(time,steeringrcI,'g')
% %plot(time,steeringrcIS,'m')
% ylabel('Inputs')
% xlabel('Time')
% legend('throttle', 'steering')
% hold off

% figure(2)
% 
% plot(time,yawratemocapI,'b')
% hold on
% plot(time,yawratemocapSI,'r')
% plot(time,yawrateimuI,'g')
% plot(time,yawrateimuSI,'m')
% hold off
% xlabel('Yaw Rate/SideSlip')
% legend('Mocap','Mocap Smooth','Imu','Imu Smooth')

if exist('leftencoder','var')
figure (3)

 plot(time,leftencoderI)
 hold on
 plot(time,leftencoderIS)
 plot(time,rightencoderI)
 plot(time,rightencoderIS)
plot(time,speedmocapSI,'c')
plot(time,longvelocitymocapSI,'r')
hold off
legend('Left','Left Smooth','Right','Right Smooth','Speed Mocap Smooth','Long V Mocap Smooth')

figure (2)

 plot(time,accellongimuCI)
 hold on
 plot(time,accellongmocapSI)
hold off
figure(4)
yyaxis('left')
hold on
plot(time,speedmocapSI,'c')
plot(time,longvelocitymocapSI,'r')
hold off
yyaxis('right')
plot(time,throttlercI)
ylim([1200,1800])

end
%% 
 sv=input('Enter 1 to save')
 end
if sv
%Define variable containing data you want to use for sysID and clear other
%variables see text file for order

processeddata(num).time=time;

processeddata(num).mocap(1).pos=posmocapSI;
processeddata(num).name=trialname;

processeddata(num).mocap(1).distance=distancemocapI;
processeddata(num).mocap(1).speed=speedmocapI;
processeddata(num).mocap(1).speed_smooth=speedmocapSI;
processeddata(num).mocap(1).longvelocity=longvelocitymocapI;
processeddata(num).mocap(1).longvelocity_smooth=longvelocitymocapSI;
processeddata(num).mocap(1).latvelocity=latvelocitymocapI;
processeddata(num).mocap(1).latvelocity_smooth=latvelocitymocapSI;
processeddata(num).mocap(1).longaccel=accellongmocapI;
processeddata(num).mocap(1).longaccel_smooth=accellongmocapSI;
processeddata(num).mocap(1).lataccel=accellatmocapI;
processeddata(num).mocap(1).lataccel_smooth=accellatmocapSI;
processeddata(num).mocap(1).heading=headingmocapI;
processeddata(num).mocap(1).heading_unwrapped=headingmocapUI;
 processeddata(num).mocap(1).heading_unwrapped_smooth=headingmocapUSI;
 processeddata(num).mocap(1).bodyslip=sideslipmocapI;
 processeddata(num).mocap(1).bodyslip_smooth=sideslipmocapSI;
 processeddata(num).mocap(1).yawrate=yawratemocapI;
 processeddata(num).mocap(1).yawrate_smooth=yawratemocapSI;
 processeddata(num).mocap(1).bodyslip_dt=sideslipmocapdtI;
 processeddata(num).mocap(1).bodyslip_dt_smooth=sideslipmocapdtSI;
 
 processeddata(num).imu(1).longaccel=accellongimuCI;
 processeddata(num).imu(1).longaccel_smooth=accellongimuCSI;
 processeddata(num).imu(1).lataccel=accellatimuCI;
 processeddata(num).imu(1).lataccel_smooth=accellatimuCSI;
 processeddata(num).imu(1).yawrate=yawrateimuI;
 processeddata(num).imu(1).yawrate_smooth=yawrateimuSI;
 processeddata(num).imu(1).heading=headingimuI;
 processeddata(num).imu(1).heading_unwrapped=headingimuUI;
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

clearvars -except processeddata* groupname filepathgroup trialnames num
save([filepathgroup,groupname],'-regexp', '^(?!(filepathgroup|groupname|num|trialnames)$).')
end
end
%% 

