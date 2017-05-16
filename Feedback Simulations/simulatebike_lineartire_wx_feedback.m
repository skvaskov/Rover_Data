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
% a=3.4;
% b=.465;
% c=6e-4;
a=0;
b=0;
c=0;
ky=1;
kpsi=1;
kx=1;
kvx=1;
kwx=.85;
kgamma=10;
P=[m,Jg,l,d,ca, Re, a, b,c,ky,kpsi,kx,kvx,kwx,kgamma];

%enter desired steering and velocity inputs (for trajectory to follow)
num=10;
gmax=5; %maximum steering angle (degrees)
vmax=1;%maximum velocity (m/s)
steps=5*ones(1,num);
garray=rand(1,num)*gmax*2*pi/180-gmax*pi/180;
vxarray=rand(1,num)*vmax;
steerfun=@(t) step(t,garray,steps);
vxfun=@(t) step(t,vxarray,steps);

%simulate results
tarray=cumsum(steps);
tspan=[0,tarray(end)];
Z0=[0;0;0;vxarray(1);vxarray(1)/Re;0;0;garray(1)];
Z0=[Z0;Z0(1:3)];
[tconst,zconst]=ode45(@(t,Z) bikeconstv(t,Z,steerfun,vxfun,[l;d]),tspan,Z0(1:3));
[tfb,zfb]=ode45(@(t,Z) bike_lineartire_wx_feedback(t,Z,steerfun,vxfun,P),tspan,Z0);

%calculate commanded inputs, tire forces
len=length(tfb);
vx0=zeros(len,1);
gammades=zeros(len,1);
gamma0=zeros(len,1);
gamma=zeros(len,1);
wxdes=zeros(len,1);
Fyf=zeros(len,1);
Fyr=zeros(len,1);
Fx=zeros(len,1);

for i=1:len
  t=tfb(i);
  x=zfb(i,1);
y=zfb(i,2);
psi=zfb(i,3);
vx=zfb(i,4);
wx=zfb(i,5);
vy=zfb(i,6);
w=zfb(i,7);
gamma=zfb(i,8);
xd=zfb(i,9);
yd=zfb(i,10);
psid=zfb(i,11);
%gamm0
gamma0(i)=steerfun(t);
vx0(i)=vxfun(t);
%error
ex=cos(psid)*(xd-x)+sin(psid)*(yd-y);
ey=-sin(psid)*(xd-x)+cos(psid)*(yd-y);
eh=psid-psi;
evx=vx0(i)-vx;
%input
gammades(i)=ky*ey+kpsi*eh+gamma0(i);
wxdes(i)=kx*ex+kvx*evx+vx0(i)/Re;
%tire forces
if vx==0 || wx==0
    Fyf(i)=0;
    Fyr(i)=0;
    Fx(i)=0;
else
Fyf(i)=ca*(gamma-(vy+(l-d)*w)/vx);
Fyr(i)=-ca*(vy-d*w)/vx;
Fx(i)=ca*(Re*wx-vx)/Re*wx;
end
end

figure
subplot(3,2,1)
plot(zfb(:,9),zfb(:,10))
hold on
plot(zfb(:,1),zfb(:,2))
xlabel('X')
ylabel('Y')
legend('Desired','Sim')
subplot(3,2,3)
plot(tfb,zfb(:,11))
hold on
plot(tfb,zfb(:,3))
xlabel('Time (s)')
ylabel('Heading (rad)')
legend('Desired','Sim')
subplot(3,2,5)
plot(tfb,vx0)
hold on
plot(tfb,zfb(:,4))
xlabel('Time (s)')
ylabel('Longitudinal Velocity (m/s)')
legend('Planned (0)','Simulated')
subplot(3,2,2)
plot(tfb,gamma0)
hold on
plot(tfb,gammades)
plot(tfb,zfb(:,8))
xlabel('Time (s)')
ylabel('Steering Angle')
legend('Planned (0)','Commanded','Simulated')
subplot(3,2,4)
plot(tfb,wxdes)
hold on
plot(tfb,zfb(:,5))
xlabel('Time (s)')
ylabel('Angular velocity rear wheel')
legend('Commanded','Simulated')
subplot(3,2,6)
plot(tfb,Fx)
hold on
plot(tfb,Fyf)
plot(tfb,Fyr)
xlabel('Time')
ylabel('Tire Forces (N)')
legend('Fx','Fyf','Fyr')
% figure
% v0c=zeros(length(tconst),1);
% g0c=zeros(length(tconst),1);
% for i=1:length(tconst)
%  t=tconst(i);
%  g0c(i)=steerfun(t);
%   v0c(i)=vxfun(t);
% end
% plot(tconst,g0c)
% hold on
% plot(tfb,gamma0)
% plot(tconst,v0c)
% plot(tfb,vx0)
% legend('gamma-const','gamma-fb','vx-const','vx-fb')
