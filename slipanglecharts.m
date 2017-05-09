clear all
load('reprocessedmarchlinedup.mat')
load('results_3_21_17.mat')
trial=processeddata10(:,1:200);
vy=trial(10,:);
vx=trial(8,:);
w=trial(26,:);
d=steeringmodel(trial(32,:));
l=results(4,4)/10;
lf=results(4,3)/10;
B=results(4,5);
C=results(4,6)/10;
D=results(4,7)*100;
%small angle approximations
lslipf=d-(vy+w*l/2).*(vx.^-1);
lslipr=(vy-w*l/2).*(vx.^-1);

%no small angle
tslipf=d-atan2(vy+w*l/2,vx);
tslipr=atan2(vy-w*l/2,vx);
Ff=D*sin(C*atan(B*tslipf));
Fr=D*sin(C*atan(B*tslipr));
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

figure
plot(tslipf,Ff);
hold on
plot(tslipr,Fr);
xlabel('Slip Angle')
ylabel('Lateral Force')
legend('Front','Rear')


