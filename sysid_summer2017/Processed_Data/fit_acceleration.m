%check 1670,1680,1690 for slip
clear all 
close all
clc
load('processed_accel_6_12.mat')
for i=1:3
figure(1)
hold on
plot(processeddata(i).time,processeddata(i).mocap.longvelocity);
plot(processeddata(i).time,processeddata(i).encoder.left);
plot(processeddata(i).time,processeddata(i).encoder.right);
figure(2)
hold on
plot(processeddata(i+3).time,processeddata(i+3).mocap.longvelocity);
plot(processeddata(i+3).time,processeddata(i+3).encoder.left);
plot(processeddata(i+3).time,processeddata(i+3).encoder.right);
figure(3)
hold on
plot(processeddata(i+6).time,processeddata(i+6).mocap.longvelocity);
plot(processeddata(i+6).time,processeddata(i+6).encoder.left);
plot(processeddata(i+6).time,processeddata(i+6).encoder.right);

end
p1690=zeros(2,3);
p1680=zeros(2,3);
p1670=zeros(2,3);
for i=1:3
    idx=find(processeddata(i).time>.5,1);
    p1670(:,i)=polyfit(processeddata(i).time(3:idx),processeddata(i).mocap.longvelocity(3:idx),1);
    idx=find(processeddata(i+3).time>.5,1);
    p1680(:,i)=polyfit(processeddata(i+3).time(3:idx),processeddata(i+3).mocap.longvelocity(3:idx),1);
    idx=find(processeddata(i+6).time>.5,1);
    p1690(:,i)=polyfit(processeddata(i+6).time(3:idx),processeddata(i+6).mocap.longvelocity(3:idx),1);
end
p1670=mean(p1670,2);
p1680=mean(p1680,2);
p1690=mean(p1690,2);
t=linspace(0,1);
figure(1)
plot(t,polyval(p1670,t),'k');
figure(2)
plot(t,polyval(p1680,t),'k');
figure(3)
plot(t,polyval(p1690,t),'k');
disp(['Accel 1670: ',num2str(p1670(1))])
disp(['Accel 1680: ',num2str(p1680(1))])
disp(['Accel 1690: ',num2str(p1690(1))])
%%
clearvars -except p*

load('processed_control1700.mat')
for i=1:12
    figure(4)
    hold on
    plot(processeddata(i).time,processeddata(i).mocap.longvelocity,'r-')
end
figure(4)
  xlabel('time')
  ylabel('velocity')
  title('fast controller')

tstop=[.17,.17,.17, .3,.3,.3,.4147,.4147,.4147,.49,.49,.49];
time=[];
vel=[];
p=zeros(2,12);
for i=1:12
    idx=find(processeddata(i).time>=tstop(i),1);
    time=[time;processeddata(i).time(1:idx)];
    vel=[vel;processeddata(i).mocap.longvelocity(1:idx)];
     p(:,i)=polyfit(processeddata(i).time(3:idx),processeddata(i).mocap.longvelocity(3:idx),1);
end

[time,I]=sort(time);
vel=vel(I);

%best fit
c=polyfit(time,vel,1);
t=linspace(0,.6);
vl=polyval(c,t);
figure(4)
hold on
plot(t,vl,'k-')
disp(['Fast Acceleration (1700): ',num2str(mean(p(1,:)))])
clearvars -except vl t c L 

load('processed_controlslow.mat')
pslow=zeros(2,6);
for i=7:12
    figure(4)
    hold on
    plot(processeddata(i).time,processeddata(i).mocap.longvelocity,'b-')
end
figure(4)
  xlabel('time')
  ylabel('velocity')
  title('fast controller')

tstop=[.38,.38,.38,.49,.49,.49];
time=[];
vel=[];
for i=7:12
    idx=find(processeddata(i).time>=tstop(i-6),1);
    time=[time;processeddata(i).time(3:idx)];
    vel=[vel;processeddata(i).mocap.longvelocity(3:idx)];
    pslow(:,i-6)=polyfit(processeddata(i).time(3:idx),processeddata(i).mocap.longvelocity(3:idx),1);
end

[time,I]=sort(time);
vel=vel(I);

%best fit
cslow=polyfit(time,vel,1);
vlslow=polyval(cslow,t);
figure(4)
hold on
plot(t,vlslow,'g-')
disp(['Slow Acceleration (1660): ',num2str(mean(pslow(1,:)))])