%% 
%clear all then
%load workspace for trial with the following contents/format:

%all imported data will be in columns

%import rover imu data with the titles for time: secsimu,nsecsimu
%angular velocity: angvX, angvY, angvZ
%linear accelaration: accX,accY,accZ

%import rc input data with titles for time: secsrc,nsecsrc
%Input1, Input2

%input mocap data time: mocaptime
%sensor positions x1,y1,z1 x2,y2,z2 x3,y3,z3

%specify start index of mocap and imu data (point before first motion):
%mocapstart, imustart
%% 

%specify the angle between the line connecting sensors 2 and 3 and the
%"striaght line" this will be used to calculate heading from mocap data and
%rotate x/y coordinate frame so the rover driving "straight" is along X axis 
gamma=0.0132;

%define rotation matrices and scaling
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

%% 
%process data and interpolate onto time scale to match inputs

%get timestamps for imu and input data
imutimeraw=secsimu+nsecsimu*10^-9;
rctimeraw=secsrc+nsecsrc*10^-9;

%determine sizes for each data set
imusize=length(secsimu);
rcsize=length(secsrc);
mocapsize=length(mocaptime);


%normalize input channels to 0, and find index of first nonzero term
%for the input channels (rcstart+1)
Input1=-(Input1-1500)';
Input2=(Input2-1500)';
k1=find(Input1,1,'first');
k2=find(Input2,1,'first');
if k1>=k2
    rcstart=k2-1;
else
    rcstart=k1-1;
end


%get position of center of rover (calculated by halfway between
%sensors 1 and 3) saved as 3xmocapsize array CPos(t)= [x(t);y(t);z(t)] and
%normalized so position before mocapstart is 0
%get heading data where heading is defined as 0 at index before mocapstart
%and is calculated by the angle between vectors for sensors 2 and 3


ns1=[x1(mocapstart);y1(mocapstart)];
ns2=[x2(mocapstart);y2(mocapstart)];
ns3=[x3(mocapstart);y3(mocapstart)];
n23=ns2-ns3;
nPos=(ns1+ns3)/2;
CPos=zeros(2,mocapsize);
headingmocap=zeros(1,mocapsize);

for i=1:mocapsize
    s1=[x1(i);y1(i)];
    s2=[x2(i);y2(i)];
    s3=[x3(i);y3(i)];
    CPos(:,i)=((s1+s3)/2-nPos);
    s23=s2-s3;
    c=cross([n23;0],[s23;0]);
    headingmocap(1,i)=sign(c(3))*atan2(norm(c),dot(n23,s23));
end

headingshift=atan2(n23(2),n23(1));
phi=headingshift-gamma;

ROT=[cos(phi) sin(phi);-sin(phi) cos(phi)];
CPos=ROT*CPos;

%calibrate acceleration data
araw=[accX';accY';accZ'];
acal=g/zavg*ROT3'*ROT2'*(A\(araw-b));
acceltanimu=acal(1,:);



%get vector for time (all interpolated data will correspond to this time
time=rctimeraw(rcstart:end)-rctimeraw(rcstart);
datasize=length(time);
%truncate and name inputs
throttlerc=Input2(rcstart:end);
steeringrc=Input1(rcstart:end); %should this be scaled so its positive and negative?

%shift and scale imu and mocap time to be zero at approriate points
timedelay=imutimeraw(imustart)-rctimeraw(rcstart);
shiftedimutime=imutimeraw-rctimeraw(rcstart)-timedelay;
shiftedmocaptime=(mocaptime-mocaptime(mocapstart))/10;

%get interpolated position, velocity, and acceleration data
acceltanimuI=zeros(1,datasize);
posmocapI=zeros(datasize,2);
headingmocapI=zeros(1,datasize);
angvZimu=zeros(1,datasize);
for i=1:datasize
    acceltanimuI(i)=interp1(shiftedimutime,acceltanimu',time(i));
    posmocapI(i,:)=interp1(shiftedmocaptime,CPos',time(i));
    headingmocapI(i)=interp1(shiftedmocaptime,headingmocap',time(i));
    angvZimu(i)=interp1(shiftedimutime,angvZ',time(i));
end
posmocapI=posmocapI';

%find heading by integrating angular velocity
headingimuI=zeros(1,datasize);
headingimuI(1)=headingmocapI(1);
for i=2:datasize
    headingimuI(i)=headingimuI(i-1)+(angvZimu(i)+angvZimu(i-1))/2*(time(i)-time(i-1));
end

%derive speed, longitudal velocity, tangential velocity from position and heading data 
speedmocapI=zeros(1,datasize);
longvelocitymocapI=zeros(1,datasize);
tanvelocitymocapI=zeros(1,datasize);
for i=1:datasize-1
    headingvector=[cos(headingmocapI(i));sin(headingmocapI(i))];
    dS=(posmocapI(1:2,i+1)-posmocapI(1:2,i));
    sn=sign(dot(dS/norm(dS),headingvector));
    speedmocapI(i)=sn*norm(dS)/(time(i+1)-time(i));
    longvelocitymocapI(i)=speedmocapI(i)*headingvector(1);
    tanvelocitymocapI(i)=speedmocapI(i)*headingvector(2);
end






%% 
%smooth data and mess with data
speedmocapIS=smooth(speedmocapI,100,'sgolay',3)';
longvelocitymocapIS=smooth(longvelocitymocapI,100,'sgolay',3)';

%derive acceleration from smooth velocity data and smooth
acceltanmocapI=zeros(1,datasize);
for i=1:datasize-1
    acceltanmocapI(i)=(speedmocapIS(i+1)-speedmocapIS(i))/(time(i+1)-time(i));
end
stanmocapI(datasize)=acceltanmocapI(datasize-1);
acceltanmocapIS=smooth(acceltanmocapI,100,'sgolay',4)';

acceltanimuIS=smooth(acceltanimuI,100,'sgolay',6)';
%smooth heading
headingmocapIS=smooth(headingmocapI,100,'sgolay',5)';

%correct acceraltion to 0
acceltanimuIC=acceltanimuI-mean(acceltanimu(1:imustart));

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
subplot(3,2,1)
plot(posmocapI(1,:),posmocapI(2,:))
xlabel('X')
ylabel('Y')
title('Position (Mocap)')

subplot(3,2,2)
plot(posmocapI(1,:),headingmocapI)
hold on
%plot(positionmocapinterp(1,:),smoothheadingmocapinterp,'r')
%plot(positionmocapinterp(1,:),headingimuinterp,'g')
xlabel('X')
ylabel('rad')
title('Heading (Mocap)')

subplot(3,2,3)
plot(time,longvelocitymocapI)
hold on
plot(time,longvelocitymocapIS,'r')
xlabel('time(s)')
ylabel('m/s')
title('Velocity (Mocap)')


subplot(3,2,4)
%plot(time,accelmocapinterp)
hold on
%plot(time,smoothaccelmocapinterp,'r')
plot(time,acceltanimuIC,'g')
plot(time,acceltanimuIS,'m')
xlabel('time(s)')
ylabel('m/s^2')
title('Acceleration (IMU)')

subplot(3,2,5)
plot(time,throttlerc,'k')
xlabel('time(s)')
title('Throttle (RC)')

subplot(3,2,6)
plot(time,steeringrc,'k')
xlabel('time(s)')
title('Steering (RC)')

%% 
%Define variable containing data you want to use for sysID and clear other
%variables (time;X;Y;heading;velocity;acceleration;throttle;steering)
processeddata=[time';posmocapI(1,:);posmocapI(2,:);...
    headingmocapI;longvelocitymocapIS;acceltanimuIC;throttlerc;steeringrc];

clearvars -except processeddata
%% 

