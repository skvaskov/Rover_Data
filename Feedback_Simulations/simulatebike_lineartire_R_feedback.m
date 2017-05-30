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
a=0;
b=0;
c=0;
ky=1;
kpsi=2;
kx=1;
kvx=10;
kgamma=0;
kgd=10;
kwx=.85;
P=[m,Jg,l,d,ca, a, b,c,ky,kpsi,kx,kvx,kgamma,kwx,kgd];

%enter desired steering and velocity inputs (for trajectory to follow)
num=5;
gmax=5; %maximum steering angle (degrees)
vmax=1;%maximum velocity (m/s)
steps=5*ones(1,num);
garray=rand(1,num)*gmax*2*pi/180-gmax*pi/180;
vxarray=rand(1,num)*vmax;
steerfun=@(t) step(t,garray,steps);
vxfun=@(t) step(t,vxarray,steps);

tarray=cumsum(steps);
tspan=[0,tarray(end)];
Z0=[0;0;0;vxarray(1);0;0;garray(1)];
Z0=[Z0;Z0(1:3)];
[tconst,zconst]=ode45(@(t,Z) bikeconstv(t,Z,steerfun,vxfun,[l;d]),tspan,Z0(1:3));
[tfb,zfb]=ode45(@(t,Z) bike_lineartire_R_feedback(t,Z,steerfun,vxfun,P),tspan,Z0);

len=length(tfb);
gamma0=zeros(len,1);
vx0=zeros(len,1);
gammades=zeros(len,1);
vxdes=zeros(len,1);
R=zeros(len,1);
Fyf=zeros(len,1);
Fyr=zeros(len,1);
for i=1:len
    t=tfb(i);
    %input vector
    gamma0(i)=steerfun(t);
    vx0(i)=vxfun(t);
    %states
x=zfb(i,1);
y=zfb(i,2);
psi=zfb(i,3);
vx=zfb(i,4);
vy=zfb(i,5);
w=zfb(i,6);
gamma=zfb(i,7);
xd=zfb(i,8);
yd=zfb(i,9);
psid=zfb(i,10);
%error
ex=cos(psid)*(xd-x)+sin(psid)*(yd-y);
ey=-sin(psid)*(xd-x)+cos(psid)*(yd-y);
eh=psid-psi;
%input
gammades(i)=ky*ey+kpsi*eh+kgamma*(gamma0(i)-gamma)+gamma0(i);
%tire forces
if vx==0
    Fyf(i)=0;
    Fyr(i)=0;
else
Fyf(i)=ca*(gamma-(vy+(l-d)*w)/vx);
Fyr(i)=-ca*(vy-d*w)/vx;
%input
vxdes(i)=kx*ex+kvx*(vx0(i)-vx)+vx0(i);
R(i)=-a-b*vx-c*vx^2+kwx*(vxdes(i)-vx);
end
end

%%
figure
subplot(3,2,1)
plot(zfb(:,8),zfb(:,9))
hold on
plot(zfb(:,1),zfb(:,2))
xlabel('X')
ylabel('Y')
subplot(3,2,3)
plot(tfb,zfb(:,10))
hold on
plot(tfb,zfb(:,3))
xlabel('Time (s)')
ylabel('Heading (rad)')
subplot(3,2,5)
plot(tfb,vx0)
hold on
plot(tfb,vxdes)
plot(tfb,zfb(:,4))
xlabel('Time (s)')
ylabel('Longitudinal Velocity (m/s)')
legend('Plan (0)','Command', 'Sim')
subplot(3,2,2)
plot(tfb,gamma0)
hold on
plot(tfb,gammades)
plot(tfb,zfb(:,7))
xlabel('Time (s)')
ylabel('Steering Angle')
legend('Plan (0)','Command', 'Sim')
subplot(3,2,4)
plot(tfb,R)
xlabel('Time (s)')
ylabel('Driving Force (N)')
subplot(3,2,6)
plot(tfb,Fyf)
hold on
plot(tfb,Fyr)
xlabel('Time')
ylabel('Tire Forces (N)')
legend('Fyf','Fyr')


