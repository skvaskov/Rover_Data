function [data] = process_data(trialnames,lineup)
%input: filepaths were files are located, cell containing list of trial names
%lineup=1 if you want to manually line up the mocap, encoder, rc, imu data
%(recommended)
filepath='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/workspaces/';
%output ntrialx1 data structure: data(i).no will have separate time vectors for mocap,
%imu, rc, and encoder data. data(i).interp will have 1 time vector that
%everyting will be linearly interpolated too
data=struct('name',{},'no',{},'interp',{});
for num=1:length(trialnames)
    load([filepath,trialnames{num},'_workspace.mat'])
if rcstart==0
    rcstart=1;
end

mocapsize=length(mocaptime);
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
araw=[accX';accY';accZ'];

acal=g/zavg*ROT3'*ROT2'*(A\(araw-b));

accellongimu=acal(1,:)';
accellatimu=acal(2,:)';
%correct acceraltion to 0
accellongimuC=accellongimu-mean(accellongimu(1:imustart));
data(num).no.imu.longaccel=accellongimuC;
accellatimuC=accellatimu-mean(accellatimu(1:imustart));
data(num).no.imu.lataccel=accellatimuC;
yawrateimu=angvZ;
pitchrateimu=angvY;
rollrateimu=angvX;
data(num).no.imu.rollrate=angvX;
data(num).no.imu.pitchrate=angvY;
data(num).no.imu.yawrate=yawrateimu;

%heading data
headingimu=orientationimu_cal(3,:)';
pitchimu=orientationimu_cal(2,:);
data(num).no.imu.pitch=pitchimu';
rollimu=orientationimu_cal(1,:);
data(num).no.imu.roll=rollimu';
%smooth imu data
accellongimuCS=smooth(imutime,accellongimuC);
data(num).no.imu.longaccel_smooth=accellongimuCS;
accellatimuCS=smooth(imutime,accellatimuC);
data(num).no.imu.lataccel_smooth=accellatimuCS;

yawrateimuS=smooth(imutime,yawrateimu);
data(num).no.imu.yawrate_smooth=yawrateimuS;
pitchrateimuS=smooth(imutime,pitchrateimu);
data(num).no.imu.pitchrate_smooth=pitchrateimuS;
rollrateimuS=smooth(imutime,rollrateimu);
data(num).no.imu.rollrate_smooth=rollrateimuS;

headingimuU=unwrap(headingimu);
data(num).no.imu.heading_unwrapped=headingimuU;
headingimuUS=smooth(imutime,headingimuU);
data(num).no.imu.heading_unwrapped_smooth=headingimuUS;
pitchimuS=smooth(imutime,pitchimu);
data(num).no.imu.pitch_smooth=pitchimuS;
rollimuS=smooth(imutime,rollimu);
data(num).no.imu.roll_smooth=rollimuS;

%mocap data
CYS=smooth(mocaptime,CPos_step(2,:));
CXS=smooth(mocaptime,CPos_step(1,:));
CZS=smooth(mocaptime,CPos_step(3,:));
CPosS=[CXS';CYS';CZS'];
headingmocap=orientationmocap_conc(3,:);
rollmocap=orientationmocap_conc(2,:);
pitchmocap=orientationmocap_conc(1,:);
data(num).no.mocap.pos=CPosS';
data(num).no.mocap.heading=headingmocap';
data(num).no.mocap.pitch=pitchmocap';
data(num).no.mocap.roll=rollmocap';

headingmocapU=unwrap(headingmocap);
hrd=diff(headingmocapU);
idxs=find(abs(hrd)>.25);
idxs=unique([idxs-1;idxs;idxs+1]);
headingmocapU(idxs)=[];
mocaptimehr=mocaptime;
mocaptimehr(idxs)=[];
headingmocapU=interp1(mocaptimehr,headingmocapU,mocaptime);
data(num).no.mocap.heading_unwrapped=headingmocapU;
headingmocapUS=smooth(mocaptime,headingmocapU);
pitchmocapS=smooth(mocaptime,pitchmocap);
rollmocapS=smooth(mocaptime,rollmocap);
data(num).no.mocap.heading_unwrapped_smooth=headingmocapUS;
data(num).no.mocap.pitch_smooth=pitchmocapS;
data(num).no.mocap.roll_smooth=rollmocapS;

distancemocap=zeros(mocapsize,1);
for i=2:mocapsize
    distancemocap(i)=norm(CPosS(:,i)-CPosS(:,i-1))+distancemocap(i-1);
end
I=repmat(eye(3),1,1,mocapsize);
ov=get_velocity([rollmocapS';pitchmocapS';headingmocapUS'],I,mocaptime);
pitchratemocap=ov(2,:);
rollratemocap=ov(1,:);
yawratemocap=ov(3,:);
data(num).no.mocap.yawrate=yawratemocap';
data(num).no.mocap.rollrate=rollratemocap';
data(num).no.mocap.pitchrate=pitchratemocap';
v=get_velocity(CPosS,Rot_conc,mocaptime);
speedmocap=sign(v(1,:)).*sqrt(sum(v(1,:).^2+v(2,:).^2,1));
data(num).no.mocap.speed=speedmocap';
sideslipmocap=atan2(v(2,:),v(1,:));
data(num).no.mocap.bodyslip=sideslipmocap';


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

data(num).no.mocap.longvelocity=longvelocitymocap';
data(num).no.mocap.latvelocity=latvelocitymocap';
data(num).no.mocap.yawratemocap_smooth=yawratemocapS;
data(num).no.mocap.pitchratemocap_smooth=pitchratemocapS;
data(num).no.mocap.rollratemocap_smooth=rollratemocapS;
data(num).no.mocap.longvelocity_smooth=longvelocitymocapS;
data(num).no.mocap.latvelocity_smooth=latvelocitymocapS;
data(num).no.mocap.speed_smooth=speedmocapS;
data(num).no.mocap.bodyslip_smooth=sideslipmocapS;
%derive acceleration from smooth velocity data 

sideslipmocapdt=get_velocity(sideslipmocapS',ones(1,1,mocapsize),mocaptime);
data(num).no.mocap.bodyslipdt=sideslipmocapdt';

accel=get_acceleration(vS,angVS,mocaptime);
accellongmocap=accel(1,:);
data(num).no.mocap.longaccel=accellongmocap';
accellatmocap=accel(2,:);
data(num).no.mocap.lataccel=accellatmocap';
%smooth data
accellongmocapS=smooth(mocaptime,accellongmocap);
accellatmocapS=smooth(mocaptime,accellatmocap);
sideslipmocapdtS=smooth(mocaptime,sideslipmocapdt);
data(num).no.mocap.lataccel_smooth=accellatmocapS;
data(num).no.mocap.longaccel_smooth=accellongmocapS;
data(num).no.mocap.bodyslipdt_smooth=sideslipmocapdtS;

%input
data(num).no.input.throttle=throttlerc;
data(num).no.input.steering=steeringrc;

%encoder
data(num).no.encoder.left=leftencoder;
data(num).no.encoder.right=rightencoder;
leftencoderS=smooth(encodertime,leftencoder);
rightencoderS=smooth(encodertime,rightencoder);
data(num).no.encoder.left_smooth=leftencoderS;
data(num).no.encoder.right_smooth=leftencoderS;

shiftedimutime=imutime-imutime(imustart);
shiftedrctime=rctime-rctime(rcstart);
shiftedmocaptime=(mocaptime-mocaptime(mocapstart));
shiftedencodertime=encodertime-encodertime(encoderstart);
if lineup~=0
    sv=0;
    while sv~=1
        mocapbias=input(['Trial ',num2str(num),', enter diff between mocap and encoder (mocap ahead is positive): ']);
        shiftedmocaptime=shiftedmocaptime-mocapbias;
        imubias=input(['Trial ',num2str(num),', enter diff between mocap and imu (mocap ahead is positive): ']);
        shiftedmocaptime=shiftedmocaptime-imubias;
        shiftedencodertime=shiftedencodertime-imubias;
        rcbias=input(['Trial ',num2str(num),',enter diff between rc and imu (imu ahead is positive): ']);
        shiftedimutime=shiftedimutime+rcbias;
        shiftedmocaptime=shiftedmocaptime+rcbias;
        shiftedencodertime=shiftedencodertime+rcbias;
        %get vector for time (all interpolated data will correspond to this time
        %cut time vector off at nearest index
       
 
       %plot results to check
       figure(1)
       plot(shiftedmocaptime,yawratemocap,'b')
       hold on
       plot(shiftedimutime,yawrateimu,'r')
       title('yawrate mocap/imu')
       hold off
       figure (2)
       yyaxis('left')
       plot(shiftedimutime,accellongimuC,'r')
       hold on
       plot(shiftedmocaptime,accellongmocap,'b')
       hold off
       yyaxis('right')
       plot(shiftedrctime,throttlerc,'g')
       ylim([1200,1800])
       title('acceleration mocap/imu/input')
       hold off

        figure (3)
        plot(shiftedencodertime,leftencoder,'m')
        hold on
        plot(shiftedencodertime,rightencoder,'c')
        plot(shiftedmocaptime,longvelocitymocap,'b')
        title('speed encoder/mocap')
        hold off

        sv=input('Enter 1 to save: ');
    end
end


  t3=find(shiftedrctime>shiftedencodertime(end),1)-1;

  t2=find(shiftedrctime>shiftedmocaptime(end),1)-1;
  t1=find(shiftedrctime>shiftedimutime(end),1)-1;
  t4=length(shiftedrctime);
  timeidx=[t1;t2;t3;t4];
  timeidx=min(timeidx);
  time=shiftedrctime(rcstart:timeidx);

data(num).no.mocap.time=shiftedmocaptime;
data(num).no.imu.time=shiftedimutime;
data(num).no.encoder.time=shiftedencodertime;
data(num).no.input.throttlerc=shiftedrctime;

 %store interpolated data
data(num).name=trialnames{num};
data(num).interp.time=time;
%mocap
data(num).interp.mocap.pos=interp1(shiftedmocaptime,CPosS',time);
data(num).interp.mocap.heading=interp1(shiftedmocaptime,headingmocap,time);
data(num).interp.mocap.heading_unwrapped_smooth=interp1(shiftedmocaptime,headingmocapUS,time);
data(num).interp.mocap.heading_unwrapped=interp1(shiftedmocaptime,headingmocapU,time);
data(num).interp.mocap.speed=interp1(shiftedmocaptime,speedmocap,time);
data(num).interp.mocap.speed_smooth=interp1(shiftedmocaptime,speedmocapS,time);
data(num).interp.mocap.distance=interp1(shiftedmocaptime,distancemocap,time);
data(num).interp.mocap.longvelocity=interp1(shiftedmocaptime,longvelocitymocap,time);
data(num).interp.mocap.longvelocity_smooth=interp1(shiftedmocaptime,longvelocitymocapS,time);
data(num).interp.mocap.latvelocity=interp1(shiftedmocaptime,latvelocitymocap,time);
data(num).interp.mocap.latvelocity_smooth=interp1(shiftedmocaptime,latvelocitymocapS,time);
data(num).interp.mocap.yawrate=interp1(shiftedmocaptime,yawratemocap,time);
data(num).interp.mocap.yawrate_smooth=interp1(shiftedmocaptime,yawratemocapS,time);       
data(num).interp.mocap.longaccel=interp1(shiftedmocaptime,accellongmocap,time);
data(num).interp.mocap.longaccel_smooth=interp1(shiftedmocaptime,accellongmocapS,time);
data(num).interp.mocap.lataccel=interp1(shiftedmocaptime,accellatmocap,time);
data(num).interp.mocap.lataccel_smooth=interp1(shiftedmocaptime,accellatmocapS,time);
%encoder

data(num).interp.encoder.left=interp1(shiftedencodertime,leftencoder,time);
data(num).interp.encoder.left_smooth=interp1(shiftedencodertime,leftencoderS,time);

data(num).interp.encoder.right=interp1(shiftedencodertime,rightencoder,time);
data(num).interp.encoder.right_smooth=interp1(shiftedencodertime,rightencoderS,time);

%input
data(num).interp.input.throttle=interp1(shiftedrctime,throttlerc,time);
data(num).interp.input.steering=interp1(shiftedrctime,steeringrc,time);
%imu
data(num).interp.imu.heading_unwrapped_smooth=interp1(shiftedimutime,headingimuUS,time);
data(num).interp.imu.heading_unwrapped=interp1(shiftedimutime,headingimuU,time);
data(num).interp.imu.yawrate=interp1(shiftedimutime,yawrateimu,time);
data(num).interp.imu.yawrate_smooth=interp1(shiftedimutime,yawrateimuS,time);       
data(num).interp.imu.lataccel=interp1(shiftedimutime,accellatimuC,time);
data(num).interp.imu.lataccel_smooth=interp1(shiftedimutime,accellatimuCS,time);
data(num).interp.imu.longaccel=interp1(shiftedimutime,accellongimuC,time);
data(num).interp.imu.longaccel_smooth=interp1(shiftedimutime,accellongimuCS,time);
clearvars -except data  num filepath trialnames lineup
end
end

