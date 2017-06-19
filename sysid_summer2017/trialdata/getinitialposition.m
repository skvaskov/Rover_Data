clear all
close all
clc
%% load calibration file, order markers correctly to generate position data
%enter trial name
trialname='pwm1A';
index=500;
%enter file path to place where raw data is stored
filepath='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/Raw_Data/pwm/';
mocapname=[filepath,trialname,'mocapdata.mat'];
load(mocapname)
%% imu
% %load imu data
imuname=[filepath,trialname,'_mavros_imu_data.csv'];
if exist(imuname,'file')
imuexcel=readtable(imuname);
accX=table2array(imuexcel(:,20));
accY=table2array(imuexcel(:,21));
accZ=table2array(imuexcel(:,22));
angvX=table2array(imuexcel(:,15));
angvY=table2array(imuexcel(:,16));
angvZ=table2array(imuexcel(:,17));
orientX=table2array(imuexcel(:,9));
orientY=table2array(imuexcel(:,10));
orientZ=table2array(imuexcel(:,11));
orientW=table2array(imuexcel(:,12));
secsimu=table2array(imuexcel(:,5));
nsecsimu=table2array(imuexcel(:,6));
imutime=secsimu+nsecsimu*10^-9;
headingimu=atan2(2*(orientW.*orientZ+orientX.*orientY),1-2*(orientY.^2+orientZ.^2));
pitch_angleimu=asin(2*(orientW.*orientY-orientZ.*orientX));
roll_angleimu=atan2(2*(orientW.*orientX+orientY.*orientZ),1-2*(orientX.^2+orientY.^2));
clearvars imuexcel
end
clearvars imuname
%%
markerorder=string(['M088';'M089';'M090';'M091';'M093';'M092';'M095';'M094']);
%reorder markers
sz=size(markerorder);
markernums=zeros(sz);
for i=1:length(markerorder)
   for j=1:sz(1)
       if markerorder(i)==string(markerdata(j).name)
           markernums(i)=j;
       end
   end
end
markerdata_reorder=markerdata(markernums);
%reordermarkerdata %[FC,FL,SL,RL,RC,RR,SR,FR]
num=size(markerdata_reorder);
num=num(2);
idx=1;
l=1;
%find marker with longest time value and smooth and interpolate the rest of the
%markers to align with it
for i=1:num
    len=length(markerdata_reorder(i).time);
    if len>l
        l=len;
        idx=i;
    end
end
mocaptime=markerdata_reorder(idx).time;
Pos=zeros(3,8,length(mocaptime));
for i=1:num
    markerdata_reorder(i).pos=interp1(markerdata_reorder(i).time,markerdata_reorder(i).pos,mocaptime);
    markerdata_reorder(i).time=mocaptime;
    Pos(:,i,:)=markerdata_reorder(i).pos';
end
%% plot markers and determine initial Position
idxend=120;
figure
% for i=1:num
%     figure(1)
%     subplot(3,1,1)
%     plot(markerdata_reorder(i).pos(1:idxend,1),markerdata_reorder(i).pos(1:idxend,3),'*')
%     hold on
%     xlabel('X')
%     ylabel('Z')
%     title('roll')
%     subplot(3,1,2)
%     plot(markerdata_reorder(i).pos(1:idxend,2),markerdata_reorder(i).pos(1:idxend,3),'*')
%     hold on
%     xlabel('Y')
%     ylabel('Z')
%     title('pitch')
%     subplot(3,1,3)
%     plot(markerdata_reorder(i).pos(1:idxend,1),markerdata_reorder(i).pos(1:idxend,2),'*')
%     hold on
%     xlabel('X')
%     ylabel('Y')
%     title('yaw')
% end
   
   P_i=mean(Pos(:,:,1:idxend),3); 
   figure(1)
    subplot(3,1,1)
   plot(P_i(1,:),P_i(3,:),'b*')
   hold on
   plot(P_i(1,1),P_i(3,1),'r*')
   plot(P_i(1,2),P_i(3,2),'g*')
   xlabel('X')
   ylabel('Z')
    subplot(3,1,2)
   plot(P_i(2,:),P_i(3,:),'b*')
   hold on
   plot(P_i(2,1),P_i(3,1),'r*')
   plot(P_i(2,2),P_i(3,2),'g*')
   xlabel('Y')
   ylabel('Z')
   subplot(3,1,3)
   plot(P_i(1,:),P_i(2,:),'b*')
   hold on
   plot(P_i(1,1),P_i(2,1),'r*')
    plot(P_i(1,2),P_i(2,2),'g*')
   xlabel('X')
   ylabel('Y')
   
   
   %% get orientation angles, etc
   P_start=P_i;
   P_icenter=mean(P_i,2);
   P_i=P_i-P_icenter;
%yaw
rFL_RL=[.47-.02088-.01890;-.01602+.02038];
rFR_RR=[.47-.01314-.01442;.01417-.011];
rFC_RC=[.47-0.01148-0.01158;.178-.172];
gamma=atan2(rFL_RL(2),rFL_RL(1));
chi=atan2(rFR_RR(2),rFR_RR(1));
phi=atan2(rFC_RC(2),rFC_RC(1));
sFL_RL=[P_i(1,2)-P_i(1,4);P_i(2,2)-P_i(2,4)];
sFR_RR=[P_i(1,8)-P_i(1,6);P_i(2,8)-P_i(2,6)];
sFC_RC=[P_i(1,1)-P_i(1,5);P_i(2,1)-P_i(2,5)];
h1=atan2(sFL_RL(2),sFL_RL(1))-gamma;
h2=atan2(sFR_RR(2),sFR_RR(1))-chi;
h3=atan2(sFC_RC(2),sFC_RC(1))-phi;
yawmocap=(h1+h2+h3)/3
O3_mocap=[cos(yawmocap),sin(yawmocap),0;-sin(yawmocap),cos(yawmocap),0;0,0,1];
P_i=O3_mocap*P_i;

%pitch
sFL_RLp=[P_i(1,2)-P_i(1,4);P_i(3,2)-P_i(3,4)];
sFR_RRp=[P_i(1,8)-P_i(1,6);P_i(3,8)-P_i(3,6)];
sFC_RCp=[P_i(1,1)-P_i(1,5);P_i(3,1)-P_i(3,5)];
p1=-atan2(sFL_RLp(2),sFL_RLp(1));
p2=-atan2(sFR_RRp(2),sFR_RRp(1));
p3=atan2(sFC_RCp(2),sFC_RCp(1));
pitchmocap=(p1+p2+p3)/3
O2_mocap=[cos(pitchmocap),0,-sin(pitchmocap);0,1,0;sin(pitchmocap),0,cos(pitchmocap)];
P_i=O2_mocap*P_i;

%roll
sFL_FR=[P_i(2,2)-P_i(2,8);P_i(3,2)-P_i(3,8)];
sSL_SR=[P_i(2,3)-P_i(2,7);P_i(3,3)-P_i(3,7)];
sRL_RR=[P_i(2,4)-P_i(2,6);P_i(3,4)-P_i(3,6)];
r1=atan2(sFL_FR(2),sFL_FR(1));
r2=atan2(sSL_SR(2),sSL_SR(1));
r3=atan2(sRL_RR(2),sRL_RR(1));
rollmocap=(r1+r2)/2
O1_mocap=[1,0,0;0,cos(rollmocap),sin(rollmocap);0,-sin(rollmocap),cos(rollmocap)];
Rmocap_i=O3_mocap'*O2_mocap'*O1_mocap';

initial_mocap=Rmocap_i\(P_start-P_icenter);
figure(1)
    subplot(3,1,1)
   plot(initial_mocap(1,:),initial_mocap(3,:),'k*')
   hold on
   plot(initial_mocap(1,1),initial_mocap(3,1),'r*')
   plot(initial_mocap(1,2),initial_mocap(3,2),'g*')
  
    subplot(3,1,2)
   plot(initial_mocap(2,:),initial_mocap(3,:),'k*')
   hold on
   plot(initial_mocap(2,1),initial_mocap(3,1),'r*')
   plot(initial_mocap(2,2),initial_mocap(3,2),'g*')
 
   subplot(3,1,3)
   plot(initial_mocap(1,:),initial_mocap(2,:),'k*')
   hold on
   plot(initial_mocap(1,1),initial_mocap(2,1),'r*')
   plot(initial_mocap(1,2),initial_mocap(2,2),'g*')
 %% get data from imu
 index=200;
 yawimu=mean(headingimu(1:index));
 rollimu=mean(roll_angleimu(1:index));
 pitchimu=mean(pitch_angleimu(1:index));
 O3_imu=[cos(yawimu),sin(yawimu),0;-sin(yawimu),cos(yawimu),0;0,0,1];
 O2_imu=[cos(pitchimu),0,-sin(pitchimu);0,1,0;sin(pitchimu),0,cos(pitchimu)];
 O1_imu=[1,0,0;0,cos(rollimu),sin(rollimu);0,-sin(rollimu),cos(rollimu)];
 Rimu_i=O3_imu'*O2_imu'*O1_imu';
 initial_imu=Rimu_i\(P_start-P_icenter);
%  figure(1)
%     subplot(3,1,1)
%    plot(initial_imu(1,:),initial_imu(3,:),'m*')
%    hold on
%    plot(initial_imu(1,1),initial_imu(3,1),'r*')
%    plot(initial_imu(1,2),initial_imu(3,2),'g*')
%   
%     subplot(3,1,2)
%    plot(initial_imu(2,:),initial_imu(3,:),'m*')
%    hold on
%    plot(initial_imu(2,1),initial_imu(3,1),'r*')
%    plot(initial_imu(2,2),initial_imu(3,2),'g*')
%  
%    subplot(3,1,3)
%    plot(initial_imu(1,:),initial_imu(2,:),'m*')
%    hold on
%    plot(initial_imu(1,1),initial_imu(2,1),'r*')
%    plot(initial_imu(1,2),initial_imu(2,2),'g*')
   %%
   clearvars -except initial_mocap initial_imu
   save('initial_positions.mat')
