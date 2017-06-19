%% 
%clear all then
%load workspace for trial with the following contents/format:
clear all
close all
clc

%%    
%enter trial number
for num=65:72
%make sv=1 if you want the step about saving stuff to run
sv=0;


%
filepathdata='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/';
load([filepathdata,'Raw_Data/yaw_rate_id/tnames.mat'])
trialname=tname{num};
groupname='yawrateid_processed.mat';
filepathgroup='~/Documents/MATLAB/Rover_Data/sysid_summer2017/';
%number in data structure

%enter file path to place where raw data is stored
load([filepathdata,trialname,'_workspace.mat'])

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
        processeddata=struct('name',{},'mocap',{},'imu',{},'encoder',{},'input',{});
    end
else
    save([filepathgroup,groupname],'groupname')
end
%%
processeddata(num).mocap=struct('x',{},'y',{},'distance',{},'speed',{},'speed_smooth',{},...
        'longvelocity',{},'longvelocity_smooth',{},'latvelocity',{},'latvelocity_smooth',{},...
        'longaccel',{},'longaccel_smooth',{},'lataccel',{},'lataccel_smooth',{},...
         'heading',{},'heading_unwrapped',{},'heading_unwrapped_smooth',{},'bodyslip',{},'bodyslip_smooth',{},...
         'yawrate',{},'yawrate_smooth',{},'bodyslip_dt',{},'bodyslip_dt_smooth',{},...
         'time',{});
 processeddata(num).imu=struct('longaccel',{},'longaccel_smooth',{},'lataccel',{},'lataccel_smooth',{},...
         'yawrate',{},'yawrate_smooth',{},'heading',{},'heading_unwrapped',{},'heading_unwrapped_smooth',{},...
         'time',{});
 processeddata(num).encoder=struct('left',{},'left_smooth',{},'right',{},'right_smooth',{},'time',{});
     processeddata(num).input=struct('throttle',{},'throttle_smooth',{},'steering',{},'steering_smooth',{},'time',{});
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


%% smooth position, heading data, imu data
CYS=smooth(mocaptime,CPos(:,2),50,'sgolay',2);
CXS=smooth(mocaptime,CPos(:,1),50,'sgolay',2);
CPosS=[CXS,CYS];
headingmocapU=unwrap(headingmocap);
hrd=diff(headingmocapU);
idxs=find(abs(hrd)>.1);
idxs=unique([idxs-1;idxs;idxs+1]);
headingmocapU(idxs)=[];
mocaptimehr=mocaptime;
mocaptimehr(idxs)=[];
headingmocapU=interp1(mocaptimehr,headingmocapU,mocaptime);
headingmocapUS=smooth(mocaptime,headingmocapU);



%smooth imu data
accellongimuCS=smooth(imutime,accellongimuC,25);
accellatimuCS=smooth(imutime,accellatimuC,25);
yawrateimuS=smooth(imutime,yawrateimu);
headingimuU=unwrap(headingimu);
headingimuUS=smooth(imutime,headingimuU);


%% get and smooth other data from mocap results
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
%  yawratemocap(1)=yaw_stepmocap(1)/(mocaptime(2)-mocaptime(1));
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
    %yawratemocap(i)=yaw_stepmocap(i)/(mocaptime(i)-mocaptime(i-1));
end

%final point
dS=CPosS(end,:)-CPosS(end-1,:);
dt=mocaptime(end)-mocaptime(end-1);
OBA=[cos(headingmocapUS(end)) sin(headingmocapUS(end));-sin(headingmocapUS(end)) cos(headingmocapUS(end))];
v(:,end)=OBA*dS'/dt;
speedmocap(end)=sign(v(1,end))*norm(v(:,end));
sideslipmocap(end)=atan2(v(2,end),v(1,end));
yawratemocap(end)=(headingmocapUS(end)-headingmocapUS(end-1))/(mocaptime(end)-mocaptime(end-1));
%yawratemocap(end)=yaw_stepmocap(end)/(mocaptime(end)-mocaptime(end-1));

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
%interpolate input and encoder
steeringrcS=smooth(rctime,steeringrc);
throttlercS=smooth(rctime,throttlerc);


leftencoderS=smooth(encodertime,leftencoder,25,'sgolay',3);
rightencoderS=smooth(encodertime,rightencoder,25,'sgolay',3);
%% shift time vectors 
%shift and scale imu and mocap time to be zero at approriate points
shiftedimutime=imutime-imutime(imustart);
shiftedrctime=rctime-rctime(rcstart);
shiftedencodertime=encodertime-imutime(imustart);
shiftedmocaptime=mocaptime;
while sv~=1

figure (1)
plot(shiftedmocaptime,yawratemocap,'b')
hold on
plot(shiftedmocaptime,yawratemocapS,'r')
plot(shiftedimutime,yawrateimu,'g')
plot(shiftedimutime,yawrateimuS,'m')
xlabel('Yaw Rate/SideSlip')
%legend('Mocap','Mocap Smooth','Imu','Imu Smooth')
hold off

figure (2)
 plot(shiftedencodertime,leftencoder)
 hold on
 plot(shiftedencodertime,leftencoderS)
 plot(shiftedencodertime,rightencoder)
 plot(shiftedencodertime,rightencoderS)
 plot(shiftedmocaptime,speedmocapS,'c')
 plot(shiftedmocaptime,longvelocitymocapS,'r')
 xlabel('encoder/speed')
 ylim([0,2.5])
%legend('Left','Left Smooth','Right','Right Smooth','Speed Mocap Smooth','Long V Mocap Smooth')
hold off

figure(3)
yyaxis left
 plot(shiftedrctime,throttlerc-1500)
yyaxis right
 plot(shiftedimutime,accellongimu)
 xlabel('accel/input')


disp(['Trial: ',num2str(num)]);
mocapbias=input('Enter shift for mocap time: ');
encoderbias=input('Enter shift for encoder (-.11): ');
rcbias=input('Enter shift for rc time: ');

shiftedmocaptime=shiftedmocaptime+mocapbias;
shiftedencodertime=shiftedencodertime+encoderbias;
shiftedrctime=shiftedrctime+rcbias;

figure (1)
plot(shiftedmocaptime,yawratemocap,'b')
hold on
plot(shiftedmocaptime,yawratemocapS,'r')
plot(shiftedimutime,yawrateimu,'g')
plot(shiftedimutime,yawrateimuS,'m')
xlabel('Yaw Rate')
%legend('Mocap','Mocap Smooth','Imu','Imu Smooth')
hold off

figure (2)
 plot(shiftedencodertime,leftencoder)
 hold on
 plot(shiftedencodertime,leftencoderS)
 plot(shiftedencodertime,rightencoder)
 plot(shiftedencodertime,rightencoderS)
 plot(shiftedmocaptime,speedmocapS,'c')
 plot(shiftedmocaptime,longvelocitymocapS,'r')
  xlabel('encoder/speed')
  ylim([0,2])
%legend('Left','Left Smooth','Right','Right Smooth','Speed Mocap Smooth','Long V Mocap Smooth')
hold off
figure(3)
 yyaxis left
 plot(shiftedrctime,throttlerc-1500)
 yyaxis right
 plot(shiftedimutime,accellongimu)
 xlabel('accel/input')

sv=input('enter 1 if this looks good,0 if not: ');

end



%% 
if sv
%Define variable containing data you want to use for sysID and clear other
%variables see text file for order
processeddata(num).mocap(1).time=shiftedmocaptime;

processeddata(num).mocap(1).x=CPosS(:,1);
processeddata(num).mocap(1).y=CPosS(:,2);
processeddata(num).mocap(1).distance=distancemocap;
processeddata(num).mocap(1).speed=speedmocap;
processeddata(num).mocap(1).speed_smooth=speedmocapS;
processeddata(num).mocap(1).longvelocity=longvelocitymocap;
processeddata(num).mocap(1).longvelocity_smooth=longvelocitymocapS;
processeddata(num).mocap(1).latvelocity=latvelocitymocap;
processeddata(num).mocap(1).latvelocity_smooth=latvelocitymocapS;
processeddata(num).mocap(1).longaccel=accellongmocap;
processeddata(num).mocap(1).longaccel_smooth=accellongmocapS;
processeddata(num).mocap(1).lataccel=accellatmocap;
processeddata(num).mocap(1).lataccel_smooth=accellatmocapS;
processeddata(num).mocap(1).heading=headingmocap;
 processeddata(num).mocap(1).heading_unwrapped=headingmocapU;
 processeddata(num).mocap(1).heading_unwrapped_smooth=headingmocapUS;
 processeddata(num).mocap(1).bodyslip=sideslipmocap;
 processeddata(num).mocap(1).bodyslip_smooth=sideslipmocapS;
 processeddata(num).mocap(1).yawrate=yawratemocap;
 processeddata(num).mocap(1).yawrate_smooth=yawratemocapS;
 processeddata(num).mocap(1).bodyslip_dt=sideslipmocapdt;
 processeddata(num).mocap(1).bodyslip_dt_smooth=sideslipmocapdtS;
 
 processeddata(num).imu(1).time=shiftedimutime;
 processeddata(num).imu(1).longaccel=accellongimuC;
 processeddata(num).imu(1).longaccel_smooth=accellongimuCS;
 processeddata(num).imu(1).lataccel=accellatimuC;
 processeddata(num).imu(1).lataccel_smooth=accellatimuCS;
 processeddata(num).imu(1).yawrate=yawrateimu;
 processeddata(num).imu(1).yawrate_smooth=yawrateimuS;
 processeddata(num).imu(1).heading=headingimu;
 processeddata(num).imu(1).heading_unwrapped=headingimuU;
 processeddata(num).imu(1).heading_unwrapped_smooth=headingimuUS;

processeddata(num).encoder(1).left=leftencoder;
processeddata(num).encoder(1).left_smooth=leftencoderS;
processeddata(num).encoder(1).right=rightencoder;
processeddata(num).encoder(1).right_smooth=rightencoderS;
processeddata(num).encoder(1).time=shiftedencodertime;

processeddata(num).input(1).throttle=throttlerc;
processeddata(num).input(1).throttle_smooth=throttlercS;
processeddata(num).input(1).steering=steeringrc;
processeddata(num).input(1).steering_smooth=steeringrcS;
processeddata(num).input(1).time=shiftedrctime;
clearvars -except processeddata* groupname filepathgroup
save([filepathgroup,groupname])
end
end

