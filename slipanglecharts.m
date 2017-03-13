clear all
load('t17.mat')
trial=imutimepsi23(:,1:200);
vy=trial(10,:);
vx=trial(8,:);
w=trial(26,:);
d=steeringmodel(trial(32,:));
l=.29;

%small angle approximations
lslipf=(vy+w*.29/2).*(vx.^-1);
lslipr=(-vy+w*.29/2).*(vx.^-1);

%real angle
tslipf=atan2(vy+w*.29/2,vx);
tslipr=atan2(-vy+w*.29/2,vx);
p1=.01;
p2=0;

figure
plot(d)
hold on

figure
plot(lslipf)
hold on
plot(lslipr)
plot(tslipf)
plot(tslipr)
plot(d)
plot(d-tslipf)
legend('Front (small approx)','Rear (small approx)','Front','Rear','Steering Angle','Steering Angle-Front Slip')
title('Slip Angles')
xlabel('time step')
ylabel('Angle (rad)')

