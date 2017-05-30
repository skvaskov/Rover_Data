clear all
close all
clc
trialname='steer1';
%enter file path to place where raw data is stored
filepath='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/Raw_Data/';
load([filepath,trialname,'mocapdata.mat'])
%run this after you already have all the mocap data loaded
%(xFC,yFC,tFC,...)
%smooth data
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
%reordermarkerdata %[FC,FL,SL,RL,RC,RR,SR,FR]
num=size(markerdata_reorder);
num=num(2);
idx=1;
l=1;
%find marker with longest time value and interpolate the rest of the
%markers to align with it
for i=1:num
    len=length(markerdata_reorder(i).time);
    if len>l
        l=len;
        idx=i;
    end
end


X=zeros(l,num);
Y=zeros(l,num);
Z=zeros(l,num);
mocaptime=markerdata_reorder(idx).time;
figure(1)
hold on
for i=1:num
    X(:,i)=interp1(markerdata_reorder(i).time,smooth(markerdata_reorder(i).time,markerdata_reorder(i).pos(:,1)),mocaptime);
    Y(:,i)=interp1(markerdata_reorder(i).time,smooth(markerdata_reorder(i).time,markerdata_reorder(i).pos(:,2)),mocaptime);
    Z(:,i)=interp1(markerdata_reorder(i).time,smooth(markerdata_reorder(i).time,markerdata_reorder(i).pos(:,3)),mocaptime);
    plot(X(:,i),Y(:,i))
    
end

plot(X(1,:),Y(1,:),'r*')
plot(X(end,:),Y(end,:),'b*')

xlim([-2 2])
ylim([-2 2])
%%

rc_all=zeros(l,2,num);
headingmocap=zeros(l,1);
rFL_RL=[.47-.02088-.01890;-.01602+.02038];
rFR_RR=[.47-.01314-.01442;.01417-.011];
rFC_RC=[.47-0.01148-0.01158;.178-.172];
gamma=atan2(rFL_RL(2),rFL_RL(1));
chi=atan2(rFR_RR(2),rFR_RR(1));
phi=atan2(rFC_RC(2),rFC_RC(1));
rc_RC=[-.01158+.47/2;.178-.345/2];
rc_RR=[-.01442+.47/2;-.011+.345/2];
rc_SR=[.238-.47/2;-.01083+.345/2];
rc_FR=[.01314-.47/2;-.01417+.345/2];
rc_FL=[.01890-.47/2;.01602-.345/2];
rc_FC=[.01148-.47/2;.172-.345/2];
rc_RL=[-.02088+.47/2;.02038-.345/2];
rc_SL=[.241-.47/2;.01346-.345/2];

for j=1:l
    sFL_RL=[X(j,2)-X(j,4);Y(j,2)-Y(j,4)];
    sFR_RR=[X(j,8)-X(j,6);Y(j,8)-Y(j,6)];
    sFC_RC=[X(j,1)-X(j,5);Y(j,1)-Y(j,5)];
    h1=atan2(sFL_RL(2),sFL_RL(1))-gamma;
    h2=atan2(sFR_RR(2),sFR_RR(1))-chi;
    h3=atan2(sFC_RC(2),sFC_RC(1))-phi;
    headingmocap(j)=(h1+h2+h3)/3;
    OBA=[cos(headingmocap(j)),sin(headingmocap(j));...
        -sin(headingmocap(j)),cos(headingmocap(j))];
    rc_all(j,:,1)=[X(j,1);Y(j,1)]+OBA'*rc_FC;
    rc_all(j,:,2)=[X(j,2);Y(j,2)]+OBA'*rc_FL;
    rc_all(j,:,3)=[X(j,3);Y(j,3)]+OBA'*rc_SL;
    rc_all(j,:,4)=[X(j,4);Y(j,4)]+OBA'*rc_RL;
    rc_all(j,:,5)=[X(j,5);Y(j,5)]+OBA'*rc_RC;
    rc_all(j,:,6)=[X(j,6);Y(j,6)]+OBA'*rc_RR;
    rc_all(j,:,7)=[X(j,7);Y(j,7)]+OBA'*rc_SR;
    rc_all(j,:,8)=[X(j,8);Y(j,8)]+OBA'*rc_FR;
end

figure(2)
plot(mocaptime,headingmocap)

for k=1:num
    figure(3)
    plot(rc_all(:,1,k),rc_all(:,2,k))
    hold on
end
figure(3)
legend(markerdata_reorder(:).name)
CPos=mean(rc_all,3);
figure(1)
plot(CPos(:,1),CPos(:,2),'k')
plot(CPos(1,1),CPos(1,2),'k*')
plot(CPos(end,1),CPos(end,2),'k*')
legend(markerdata_reorder(:).name,'start','finish','CPos')