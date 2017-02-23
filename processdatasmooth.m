%% 
%clear all then
%load workspace for trial with the following contents/format:
clear all

load(['trial24_4-jan-2017.mat'])

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

%specify 'chop' this will cut off the beginning of the mocap data prior to
%recorded (needed for smoothing)
check=abs(x1)>.0000001;
chop=find(check,1);
endchop=length(x1);
%this version will calculate heading, etc. from smoothed position data
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
%%
%smooth and cut raw mocap data with respect to time

x1=x1(chop:endchop);
x2=x2(chop:endchop);
x3=x3(chop:endchop);
y1=y1(chop:endchop);
y2=y2(chop:endchop);
y3=y3(chop:endchop);
z1=z1(chop:endchop);
z2=z2(chop:endchop);
z3=z3(chop:endchop);
mocaptime=0:1/60:(length(mocaptime)-1)/60;
%mocaptime=mocaptime/10;
mocaptime=mocaptime(chop:endchop);
mocapsize=length(mocaptime);
mocapstart=mocapstart-chop;

%%
x1S=smooth(mocaptime,x1,100,'sgolay',2);
x2S=smooth(mocaptime,x2,100,'sgolay',2);
x3S=smooth(mocaptime,x3,100,'sgolay',2);

y1S=smooth(mocaptime,y1,50,'sgolay',2);
y2S=smooth(mocaptime,y2,50,'sgolay',2);
y3S=smooth(mocaptime,y3,50,'sgolay',2);

subplot(2,1,1)
plot(mocaptime,x1S,'r')
hold on
plot(mocaptime,x1,'r--')
plot(mocaptime,x2S,'b')
plot(mocaptime,x2,'b--')
plot(mocaptime,x3S,'g')
plot(mocaptime,x3,'g--')
hold off

subplot(2,1,2)
plot(mocaptime,y1S,'r')
hold on
plot(mocaptime,y1,'r--')
plot(mocaptime,y2S,'b')
plot(mocaptime,y2,'b--')
plot(mocaptime,y3S,'g')
plot(mocaptime,y3,'g--')
hold off



%%
ns1=[x1S(mocapstart);y1S(mocapstart)];
ns2=[x2S(mocapstart);y2S(mocapstart)];
ns3=[x3S(mocapstart);y3S(mocapstart)];
n23=ns2-ns3;
nPos=(ns1+ns3)/2;
CPos=zeros(2,mocapsize);
headingmocap=zeros(1,mocapsize);

for i=1:mocapsize
    s1=[x1S(i);y1S(i)];
    s2=[x2S(i);y2S(i)];
    s3=[x3S(i);y3S(i)];
    CPos(:,i)=((s1+s3)/2-nPos);
    s23=s2-s3;
    c=cross([n23;0],[s23;0]);
    headingmocap(1,i)=sign(c(3))*atan2(norm(c),dot(n23,s23));
end

headingshift=atan2(n23(2),n23(1));
phi=headingshift-gamma;

ROT=[cos(phi) sin(phi);-sin(phi) cos(phi)];
CPos=ROT*CPos;
%%
%smooth position  and heading data
CYS=smooth(mocaptime,CPos(2,:),30,'sgolay',2);
CXS=smooth(mocaptime,CPos(1,:),50,'sgolay',2);
CPosS=[CPos(1,:);CYS'];
headingmocapS=smooth(mocaptime,headingmocap,30,'sgolay',2);

subplot(2,1,1)
plot(CPos(1,:),CPos(2,:))
hold on
plot(CPosS(1,:),CPosS(2,:))
hold off
subplot(2,1,2)
plot(mocaptime,headingmocap)
hold on
plot(mocaptime,headingmocapS)
hold off



%%
%calibrate acceleration data
araw=[accX';accY';accZ'];
acal=g/zavg*ROT3'*ROT2'*(A\(araw-b));
accellongimu=acal(1,:);
accellatimu=acal(2,:);
%correct acceraltion to 0
accellongimuC=accellongimu-mean(accellongimu(1:imustart));
accellatimuC=accellatimu-mean(accellatimu(1:imustart));

%get vector for time (all interpolated data will correspond to this time
time=rctimeraw(rcstart:end)-rctimeraw(rcstart)';
datasize=length(time);

%truncate and name inputs
throttlerc=Input2(rcstart:end);
steeringrc=Input1(rcstart:end);
throttlercS=smooth(throttlerc)';
steeringrcS=smooth(steeringrc)';

%shift and scale imu and mocap time to be zero at approriate points

timedelay=imutimeraw(imustart)-rctimeraw(rcstart);
shiftedimutime=imutimeraw-rctimeraw(rcstart)-timedelay;
shiftedmocaptime=(mocaptime-mocaptime(mocapstart));

%get interpolated position, and acceleration data
accellongimuCI=zeros(1,datasize);
accellatimuCI=zeros(1,datasize);
posmocapI=zeros(datasize,2);
headingmocapI=zeros(1,datasize);
headingmocapSI=zeros(1,datasize);
yawrateimuI=zeros(1,datasize);
%%
for i=1:datasize
    accellongimuCI(i)=interp1(shiftedimutime,accellongimuC',time(i));
    accellatimuCI(i)=interp1(shiftedimutime,accellatimuC',time(i));
    posmocapI(i,:)=interp1(shiftedmocaptime,CPosS',time(i));
    headingmocapI(i)=interp1(shiftedmocaptime,headingmocap',time(i));
    headingmocapSI(i)=interp1(shiftedmocaptime,headingmocapS',time(i));
    yawrateimuI(i)=interp1(shiftedimutime,angvZ',time(i));
    if isnan(posmocapI(i,1)) || isnan(yawrateimuI(i))
        datasize=i-1;
        accellongimuCI=accellongimuCI(1:datasize);
        accellatimuCI=accellatimuCI(1:datasize);
        posmocapI=posmocapI(1:datasize,:);
        headingmocapI=headingmocapI(1:datasize);
        headingmocapSI=headingmocapSI(1:datasize);
        yawrateimuI=yawrateimuI(1:datasize);
        time=time(1:datasize);
        throttlerc=throttlerc(1:datasize);
        steeringrc=steeringrc(1:datasize);
        throttlercS=throttlercS(1:datasize);
        steeringrcS=steeringrcS(1:datasize);
        str=[num2str(i) ' nan']
        break
    end
        
end
posmocapI=posmocapI';

accellongimuCIS=smooth(accellongimuCI,25,'sgolay',3)';
accellatimuCIS=smooth(accellatimuCI,25,'sgolay',3)';
yawrateimuIS=smooth(yawrateimuI,25,'sgolay',3)';
%% get and smooth other data from mocap results

%% get and smooth other data from mocap results

%derive speed (signed similar to long velocity), longitudal velocity, distance, lateral velocity from smooth position and heading data 
speedmocapI=zeros(1,datasize);
longvelocitymocapI=zeros(1,datasize);
latvelocitymocapI=zeros(1,datasize);
distancemocapI=zeros(1,datasize);
sideslipmocapI=zeros(1,datasize);
yawratemocapI=zeros(1,datasize);
v=zeros(2,datasize);

for i=1:datasize-1
    dS=(posmocapI(:,i+1)-posmocapI(:,i));
    dt=time(i+1)-time(i);
    distancemocapI(i+1)=distancemocapI(i)+norm(dS);
    O3=[cos(headingmocapSI(i)) sin(headingmocapSI(i));-sin(headingmocapSI(i)) cos(headingmocapSI(i))];
    v(:,i)=O3*dS/dt;
    longvelocitymocapI(i)=v(1,i);
    latvelocitymocapI(i)=v(2,i);
    speedmocapI(i)=sign(v(1,i))*norm(v(:,i));
    sideslipmocapI(i)=atan2(v(2,i),v(1,i));
    yawratemocapI(i)=(headingmocapSI(i+1)-headingmocapSI(i))/(time(i+1)-time(i));
end
yawratemocapIS=smooth(yawratemocapI)';


speecmocapI(datasize)=speedmocapI(datasize-1);
longvelocitymocapI(datasize)=longvelocitymocapI(datasize-1);
latvelocitymocapI(datasize)=latvelocitymocapI(datasize-1);
sideslipmocapI(datasize)=sideslipmocapI(datasize-1);
yawratemocapI(datasize)=yawratemocapI(datasize-1);

speedmocapIS=smooth(speedmocapI)';
longvelocitymocapIS=smooth(longvelocitymocapI)';
latvelocitymocapIS=smooth(latvelocitymocapI)';
sideslipmocapIS=smooth(sideslipmocapI)';


%derive acceleration from smooth velocity data 
accellongmocapI=zeros(1,datasize);
accellatmocapI=zeros(1,datasize);
sideslipmocapdtI=zeros(1,datasize);
for i=1:datasize-1
    dVdt=(v(:,i+1)-v(:,i))/(time(i+1)-time(i));
    a=[dVdt;0]+cross([0;0;yawratemocapIS(i)],[v(:,i);0]);
    accellongmocapI(i)=a(1);
    accellatmocapI(i)=a(2);
    sideslipmocapdtI(i)=(sideslipmocapIS(i+1)-sideslipmocapIS(i))/(time(i+1)-time(i));
end
accellongmocapI(datasize)=accellongmocapI(datasize-1);
accellatmocapI(datasize)=accellatmocapI(datasize-1);
sideslipmocapdtI(datasize)=sideslipmocapdtI(datasize-1);

accellongmocapIS=smooth(accellongmocapI,25,'sgolay',2)';
accellatmocapIS=smooth(accellatmocapI,25,'sgolay',2)';
sideslipmocapdtIS=smooth(sideslipmocapdtI,50,'sgolay',2)';


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
plot(time,posmocapI(1,:),'b')
hold on
plot(time,posmocapI(2,:),'g')
ylabel('position')
hold off

subplot(4,2,2)
plot(time,distancemocapI,'k')
ylabel('Distance')

subplot(4,2,3)
plot(time,headingmocapI,'b')
hold on
plot(time,headingmocapSI,'r')
plot(time,sideslipmocapI,'g')
plot(time,sideslipmocapIS,'m')
ylabel('Heading/Sideslip')
ylim([-1 1])
hold off

subplot(4,2,4)
plot(time,speedmocapI,'k')
hold on
plot(time,speedmocapIS,'c')
plot(time,longvelocitymocapI,'b')
plot(time,longvelocitymocapIS,'r')
plot(time,latvelocitymocapI,'g')
plot(time,latvelocitymocapI,'m')
ylabel('Veocity')
hold off

subplot(4,2,5)
plot(time,accellongmocapI,'b')
hold on
plot(time,accellongmocapIS,'r')
plot(time,accellatmocapI,'g')
plot(time,accellatmocapIS,'m')
ylabel('Acceleration (Mocap)')
hold off

subplot(4,2,6)
plot(time,accellongimuCI,'b')
hold on
plot(time,accellongimuCIS,'r')
plot(time,accellatimuCI,'g')
plot(time,accellatimuCIS,'m')
ylabel('Acceleration (imu)')
hold off

subplot(4,2,7)
plot(time,sideslipmocapdtI,'k')
hold on
plot(time,sideslipmocapdtIS,'c')
plot(time,yawratemocapI,'b')
plot(time,yawratemocapIS,'r')
plot(time,yawrateimuI,'g')
plot(time,yawrateimuIS,'m')
xlabel('Yaw Rate/SideSlip (imu is g)')
ylim([-1.5 1.5])
hold off

subplot(4,2,8)
plot(time,throttlerc,'b')
hold on
plot(time,throttlercS,'r')
plot(time,steeringrc,'g')
plot(time,steeringrcS,'m')
xlabel('Inputs')
hold off

%% 
%Define variable containing data you want to use for sysID and clear other
%variables see text file for order
processeddata24=[time';posmocapI(1,:);posmocapI(2,:);distancemocapI;...
    speedmocapI;speedmocapIS;longvelocitymocapI;longvelocitymocapIS;latvelocitymocapI;latvelocitymocapIS;...
    accellongimuCI;accellongimuCIS;accellatimuCI;accellatimuCIS;...
    accellongmocapI;accellongmocapIS;accellatmocapI;accellatmocapIS;...
    headingmocapI;headingmocapSI;sideslipmocapI;sideslipmocapIS;...
    yawrateimuI;yawrateimuIS;yawratemocapI;yawratemocapIS;sideslipmocapdtI;sideslipmocapdtIS;...
    throttlerc;throttlercS;steeringrc;steeringrcS];

clearvars -except processeddata*
save('smoothdata100.mat','processeddata*','-append')
%% 

