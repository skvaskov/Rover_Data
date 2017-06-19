
clc
speed=[1670,1680,1690];

%for ispeed=1:length(speed)

    for it=1:12
%enter file path to place where raw data is stored
filepath='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/Raw_Data/controller_id/';
%enter trial name
trialname=trialnames{it};
%% input
%load RC data
rcname=[filepath,trialname,'_mavros_rc_in.csv'];
if exist(rcname,'file')
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
clearvars rcexcel split row i2 i1 i channel 
end
clearvars rcname
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
orientationimu(3,:)=atan2(2*(orientW.*orientZ+orientX.*orientY),1-2*(orientY.^2+orientZ.^2));
orientationimu(2,:)=asin(2*(orientW.*orientY-orientZ.*orientX));
orientationimu(1,:)=atan2(2*(orientW.*orientX+orientY.*orientZ),1-2*(orientX.^2+orientY.^2));
load('imucal_circles.mat')
orientationimu_cal=regParams.R*unwrap(orientationimu,[],2)+regParams.t;

clearvars imuexcel
end
clearvars imuname
%% encoder
%load encoder data
enname=[filepath,trialname,'_encoder_data.csv'];
if exist(enname,'file')
enexcel=readtable(enname);
leftencoder=table2array(enexcel(:,8));
rightencoder=table2array(enexcel(:,9));
secsencoder=table2array(enexcel(:,5));
nsecsencoder=table2array(enexcel(:,6));
encodertime=secsencoder+nsecsencoder*10^-9;
clearvars enexcel enname
end
clearvars enname
%% mocap
% %load mocap data

load('initial_positions.mat')
load([filepath,trialname,'mocapdata.mat'])
clearvars itf
%specify order of markers (counter clockwise starting with front center)
% markerorder= [FC,FL,SL,RL,RC,RR,SR,FR]
markerorder=string(['M088';'M089';'M090';'M091';'M093';'M092';'M095';'M094']);
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
[Rot_step,Rot_conc,Rot_global,trans_step,trans_global,mocaptime,error_step,error_global]=rigidbodyfit(markerdata_reorder,initial_mocap,[8,4]);
[orientationmocap_step]=get_orientation(Rot_step);
[orientationmocap_conc]=get_orientation(Rot_conc);
[orientationmocap_global]=get_orientation(Rot_global);

%find offset of centroid from center
     rc(:,5)=[-.01158+.47/2;.178-.345/2;0]; %RC
     rc(:,6)=[-.01442+.47/2;-.011+.345/2;0]; %RR
     rc(:,7)=[.238-.47/2;-.01083+.345/2;0]; %SR
     rc(:,8)=[.01314-.47/2;-.01417+.345/2;0]; %FR
     rc(:,2)=[.01890-.47/2;.01602-.345/2;0];%FL
     rc(:,1)=[.01148-.47/2;.172-.345/2;0];%FC
     rc(:,4)=[-.02088+.47/2;.02038-.345/2;0]; %RL
     rc(:,3)=[.241-.47/2;.01346-.345/2;0];%SL
     centroid_offset=mean(rc,2);
     CPos_step=get_position(trans_step(:,1),Rot_step,trans_step);
     CPos_step=centroid_topoint(squeeze(CPos_step),Rot_conc,centroid_offset);
     CPos_global=centroid_topoint(trans_global,Rot_global,centroid_offset);


clearvars centroid_offset
%% set start index
%select imu and mocap start points (index before first motion)


figure(3)
plot(accX)
imustart=input([trialname,' enter imu start index:']);

if exist('leftencoder','var')
   encoderstart=find((leftencoder>0).*(rightencoder>0),1)-1;
end
%normalize input channels to 0, and find index of first nonzero term
%for the input channels (rcstart+1)
rcstart=find(throttlerc-1500>0,1)-1;


diffC=diff(CPos_step,1,2);
normdiffC=sqrt(sum(diffC.^2,1));
%find mocapstart
figure(1)
subplot(2,1,1)
plot(CPos_step(1,:))
subplot(2,1,2)
plot(CPos_step(2,:))
figure(2)
plot(normdiffC)
mocapstart=input([trialname,' enter motion capture start index:']);
%% save
%clear excess variables and save workspace
 clearvars filepath k1 k2  e1 e2 markerdata markerorder nsecs* secs* diffC normdiffC i j sz rc

 save([trialname,'_workspace.mat'],'-regexp', '^(?!(it|ispeed|speed|trialnames)$).')
 clearvars -except it ispeed speed trialnames
    end
    
%end
