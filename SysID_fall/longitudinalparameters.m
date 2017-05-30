%fit data for coastdown parameters
clear all
clc
load('smoothdata100imutime.mat');

trial=processeddata2;
t=trial(1,155:185)-trial(1,155);
y=trial(6,155:185);
% 
F=@(x,xdata)sqrt(x(1)/x(2))*tan(atan(sqrt(x(2)/x(1))*y(1))-sqrt(x(2)*x(1))*xdata);
x0=[.1 .0005];
[x,resnorm,~,exitflag,output]=lsqcurvefit(F,x0,t,y);
figure
plot(t,y,'r.')
hold on
plot(t,F(x,t));

xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('3 b)')
legend('Actual Data','Fitted Curve')
