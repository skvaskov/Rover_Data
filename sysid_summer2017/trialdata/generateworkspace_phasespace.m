clear all
close all
clc
%enter trial name
trialname='straight1';
%enter file path to place where raw data is stored
filepath='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/Raw_Data/';

%% input
%load RC data
rcname=[filepath,trialname,'_mavros_rc_in.csv'];
rcexcel=readtable(rcname);
secsrc=table2array(rcexcel(:,5));
nsecsrc=table2array(rcexcel(:,6));
channel=table2array(rcexcel(:,9));
i1=cell(length(secsrc),1);
i2=cell(length(secsrc),1);
for i=1:length(secsrc)
    row=char(channel(i));
    split=strsplit(row,{'[',', '});
    i1(i)=split(2);
    i2(i)=split(4);
end
steeringrc=str2num(char(i1));
throttlerc=str2num(char(i2));
rctime=secsrc+nsecsrc*10^-9;
clearvars rcexcel split row i2 i1 i channel rcname

%% imu
% %load imu data
imuname=[filepath,trialname,'_mavros_imu_data.csv'];
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
clearvars imuexcel imuname
%% encoder
%load encoder data
enname=[filepath,trialname,'_encoder_data.csv'];
enexcel=readtable(enname);
leftencoder=table2array(enexcel(:,8));
rightencoder=table2array(enexcel(:,9));
secsencoder=table2array(enexcel(:,5));
nsecsencoder=table2array(enexcel(:,6));
encodertime=secsencoder+nsecsencoder*10^-9;
clearvars enexcel enname
%% mocap
% %load mocap data
load([filepath,trialname,'mocapdata.mat'])
%specify order of markers (counter clockwise starting with front center)
% markerorder= [FC,FL,SL,RL,RC,RR,SR,FR]
markerorder=string(['M088';'M089';'M090';'M091';'M093';'M092';'M095';'M094']);
[CPos,headingmocap,mocaptime,X,Y,Z]=getcenterandheading(markerdata,markerorder);

%% set start index
%select imu and mocap start points (index before first motion) 
imustart=69;
encoderstart=17;
mocapstart=284;

%normalize input channels to 0, and find index of first nonzero term
%for the input channels (rcstart+1)
k1=find(steeringrc-1500,1,'first');
k2=find(throttlerc-1500,1,'first');
if k1>=k2
    rcstart=k2-1;
else
    rcstart=k1-1;
end

%% save
%clear excess variables and save workspace
 clearvars filepath k1 k2  markerdata markerorder nsecs* secs*
save([trialname,'_workspace.mat'])