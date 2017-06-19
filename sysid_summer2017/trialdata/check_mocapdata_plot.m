clear all
close all
clc
%enter trial name
trialname='circles25_5_31';
%enter file path to place where raw data is stored
filepath='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/Raw_Data/';
% %load mocap data
mocapname=[filepath,trialname,'mocapdata.mat'];
load('initial_positions.mat')

load(mocapname)
%specify order of markers (counter clockwise starting with front center)
% markerorder= [FC,FL,SL,RL,RC,RR,SR,FR]
markerorder=string(['M088';'M089';'M090';'M091';'M093';'M092';'M095';'M094']);
[CPos_old,headingmocap_old,mocaptime_old,X,Y,Z]= getcenterandheading(markerdata,markerorder);
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
  %vectors pointing from each marker to center of rover
    rc(:,5)=[-.01158+.47/2;.178-.345/2;0]; %RC
    rc(:,6)=[-.01442+.47/2;-.011+.345/2;0]; %RR
    rc(:,7)=[.238-.47/2;-.01083+.345/2;0]; %SR
    rc(:,8)=[.01314-.47/2;-.01417+.345/2;0]; %FR
    rc(:,2)=[.01890-.47/2;.01602-.345/2;0];%FL
    rc(:,1)=[.01148-.47/2;.172-.345/2;0];%FC
    rc(:,4)=[-.02088+.47/2;.02038-.345/2;0]; %RL
    rc(:,3)=[.241-.47/2;.01346-.345/2;0];%SL
centroid_offset=mean(rc,2);
[Rot_step,Rot_conc,Rot_global,trans_step,trans_global,mocaptime,error_step,error_global]=rigidbodyfit(markerdata_reorder,initial_mocap,[8,4]);
[orientation_step]=get_orientation(Rot_step);
[orientation_conc]=get_orientation(Rot_conc);
[orientation_global]=get_orientation(Rot_global);
 CPos=get_position(trans_step(:,1),Rot_step,trans_step);
 CPos=centroid_topoint(squeeze(CPos),Rot_conc,centroid_offset);
 CPos_global=centroid_topoint(trans_global,Rot_global,centroid_offset);
 en=[1,500,1000];
% 
figure
  plot(CPos_old(1:en(end),1),CPos_old(1:en(end),2))
 hold on
 plot(CPos(1,1:en(end)),CPos(2,1:en(end)))
 plot(CPos_global(1,:),CPos_global(2,:))
 plot(CPos(1,en(end)),CPos(2,en(end)),'b*')
 plot(CPos(1,en(1)),CPos(2,en(1)),'g*')
 plot(X(en,2),Y(en,2),'k*')
 plot(X(en,4),Y(en,4),'k*')
 plot(CPos(1,en),CPos(2,en),'r*')
 plot(X(en,6),Y(en,6),'k*')
 plot(X(en,8),Y(en,8),'k*')
 legend('point','rigid body_step','rigid body_global (centroid of markers)')
 xlabel('X')
 ylabel('Y')
 xlim([-3 3])
 ylim([-3 3])
figure
plot(mocaptime_old,headingmocap_old)
hold on
plot(mocaptime,orientation_global(3,:))
plot(mocaptime(en),orientation_global(3,en),'r*')
legend('point','rigid body')
xlabel('time(s)')
ylabel('heading')
% 
figure
subplot(2,1,1)
plot(mocaptime,orientation_conc(3,:))
hold on
plot(mocaptime,orientation_global(3,:))
hold off
legend('Conc','Global')
title('yaw')
subplot(2,1,2)
plot(mocaptime,orientation_step(3,:))

figure
subplot(2,1,1)
plot(mocaptime,orientation_conc(2,:))
hold on
plot(mocaptime,orientation_global(2,:))
hold off
legend('Conc','Global')
title('pitch')
subplot(2,1,2)
plot(mocaptime,orientation_step(2,:))

figure
subplot(2,1,1)
plot(mocaptime,orientation_conc(1,:))
hold on
plot(mocaptime,orientation_global(1,:))
hold off
legend('Conc','Global')
title('roll')
subplot(2,1,2)
plot(mocaptime,orientation_step(1,:))
figure
subplot(2,1,1)
plot(mocaptime(2:end),diff(mocaptime))
xlabel('time')
ylabel('diff(time)')
subplot(2,1,2)
plot(mocaptime,error_step)
hold on
plot(mocaptime,error_global)
xlabel('time')
ylabel('lsq error')
legend('step','global')
