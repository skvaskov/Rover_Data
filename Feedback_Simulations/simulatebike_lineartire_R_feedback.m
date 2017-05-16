%simulate trajectories for bike model with linear tire force

clear all
close all
clc

m=2.759;
Jg=.1733;
l=.3;
d=.3-.1585;
ca=24;
Re=sqrt(.116^2-.01^2);
a=3.4;
b=.465;
c=.6e-4;
ky=50;
kpsi=50;
kx=50;
kvx=10;
kwx=.85;
P=[m,Jg,l,d,ca, Re, a, b,c,ky,kpsi,kx,kvx,kwx];
%center desired steering and velocity (trajectory paramters)
gamma0=2*pi/180;
vx0=.3;

tspan=[0,100];
Z0=[0;0;0;.1;0;0];
Z0=[Z0;Z0(1:3)];
[tconst,zconst]=ode45(@(t,Z) bikeconstv(t,Z,[gamma0;vx0],[l;d]),tspan,Z0(1:3));
[tfb,zfb]=ode45(@(t,Z) bike_lineartire_R_feedback(t,Z,[gamma0;vx0],P),tspan,Z0);

%error
ex=cos(zfb(:,9)).*(zfb(:,7)-zfb(:,1))+sin(zfb(:,9)).*(zfb(:,8)-zfb(:,2));
ey=-sin(zfb(:,9)).*(zfb(:,7)-zfb(:,1))+cos(zfb(:,9)).*(zfb(:,8)-zfb(:,2));
eh=zfb(:,9)-zfb(:,3);
evx=vx0-zfb(:,4);
%input

gamma=ky*ey+kpsi*eh+gamma0;

%tire forces
Fyf=ca*(gamma-(zfb(:,5)+(l-d)*zfb(:,6)).*(zfb(:,4).^-1));
Fyr=-ca*(zfb(:,5)-d*zfb(:,6)).*(zfb(:,4).^-1);

%input
vxdes=kx*ex+kvx*evx;


figure
subplot(3,2,1)
plot(zconst(:,1),zconst(:,2))
hold on
plot(zfb(:,1),zfb(:,2))
xlabel('X')
ylabel('Y')
subplot(3,2,3)
plot(tconst,zconst(:,3))
hold on
plot(tfb,zfb(:,3))
xlabel('Time (s)')
ylabel('Heading (rad)')
subplot(3,2,5)
plot(tspan,[vx0,vx0])
hold on
plot(tfb,zfb(:,4))
xlabel('Time (s)')
ylabel('Longitudinal Velocity (m/s)')
subplot(3,2,2)
plot(tspan,[gamma0 gamma0])
hold on
plot(tfb,gamma)
xlabel('Time (s)')
ylabel('Steering Angle')
subplot(3,2,4)
plot(tfb,vxdes)
hold on
plot(tfb,zfb(:,4))
xlabel('Time (s)')
ylabel('Long velocity')
legend('Commanded','Simulated')
subplot(3,2,6)
plot(tfb,Fyf)
hold on
plot(tfb,Fyr)
xlabel('Time')
ylabel('Tire Forces (N)')
legend('Fyf','Fyr')


