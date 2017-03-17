clear all
load('reprocessedmarchlinedup.mat')
trial=processeddata17(:,1:200);
vy=trial(10,:);
vx=trial(8,:);
w=trial(26,:);
d=steeringmodel(trial(32,:));
l=.29;

%small angle approximations
lslipf=d-(vy+w*.29/2).*(vx.^-1);
lslipr=(-vy+w*.29/2).*(vx.^-1);

%real angle
tslipf=d-atan2(vy+w*.29/2,vx);
tslipr=atan2(-vy+w*.29/2,vx);
p1=-.00111500434719498;
p2=0.08;
scaledsteering=p1*trial(32,:)+p2;

figure
plot(d)
hold on
plot(scaledsteering)
legend('Steering Model','Scaled Input')
hold on

figure
plot(lslipf)
hold on
plot(lslipr)
plot(tslipf)
plot(tslipr)
legend('Front (small approx)','Rear (small approx)','Front','Rear')
title('Slip Angles')
xlabel('time step')
ylabel('Angle (rad)')

