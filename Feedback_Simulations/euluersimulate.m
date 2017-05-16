clear all 
close all
clc
load('randdata.mat')

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

P=[ky,kpsi,kx,kvx,kwx];
Pn=[m,Jg,l,d,ca, Re, a, b,c,ky,kpsi,kx,kvx,kwx];

time=tfb;
len=length(time);
Zgt=zeros(7,length(time));
Zgt(:,1)=zfb(1,1:7)';
Zn=zeros(10,length(time));
Zn(:,1)=zfb(1,:)';
steerfun=@(t)interp1(tfb,gamma0,t);
vxfun=@(t)interp1(tfb,vx0,t);

for i=2:len
 
    dt=time(i)-time(i-1);
    U=[steerfun(time(i-1));vxfun(time(i-1));zfb(i-1,1:3)'];
    P=ones(1,4);
    dfdtgt=bike_lineartire_gaintuning(Zgt(:,i-1),U,P);
    dfdtn=bike_lineartire_wx_feedback(time(i-1),Zn(:,i-1),steerfun,vxfun,Pn);
    Zgt(:,i)=Zgt(:,i-1)+dt*dfdtgt;
    Zn(:,i)=Zn(:,i-1)+dt*dfdtn;
end
%calculate commanded inputs, tire forces
Zn=Zn';
vx0=zeros(len,1);
gamma0=zeros(len,1);
gamma=zeros(len,1);
wxdes=zeros(len,1);
Fyf=zeros(len,1);
Fyr=zeros(len,1);
Fx=zeros(len,1);

for i=1:len
  x=Zn(i,1);
y=Zn(i,2);
psi=Zn(i,3);
vx=Zn(i,4);
wx=Zn(i,5);
vy=Zn(i,6);
w=Zn(i,7);
xd=Zn(i,8);
yd=Zn(i,9);
psid=Zn(i,10);
%gamm0
gamma0(i)=steerfun(time(i));
vx0(i)=vxfun(time(i));
%error
ex=cos(psid)*(xd-x)+sin(psid)*(yd-y);
ey=-sin(psid)*(xd-x)+cos(psid)*(yd-y);
eh=psid-psi;
evx=vx0(i)-vx;
%input
gamma(i)=ky*ey+kpsi*eh+gamma0(i);
if gamma>.8
    gamma=.8;
elseif gamma<-.8
    gamma=-.8;
end
wxdes(i)=kx*ex+kvx*evx+vx0(i)/Re;
if wxdes>100
    wxdes=100;
end

%tire forces
if vx==0 || wx==0
    Fyf(i)=0;
    Fyr(i)=0;
    Fx(i)=0;
else
Fyf(i)=ca*(gamma(i)-(vy+(l-d)*w)/vx);
Fyr(i)=-ca*(vy-d*w)/vx;
Fx(i)=ca*(Re*wx-vx)/Re*wx;
end
end
Zgt=Zgt';
figure
subplot(3,2,1)
plot(Zn(:,8),Zn(:,9))
hold on
plot(Zn(:,1),Zn(:,2))
plot(Zgt(:,1),Zgt(:,2))
xlabel('X')
ylabel('Y')
legend('Ode Desired','Ode Sim','Gain Train')
subplot(3,2,3)
plot(time,Zn(:,10))
hold on
plot(time,Zn(:,3))
plot(time,Zgt(:,3))
xlabel('Time (s)')
ylabel('Heading (rad)')
legend('Desired','Sim','Gain Train')
subplot(3,2,5)
plot(time,vx0)
hold on
plot(time,Zn(:,4))
xlabel('Time (s)')
ylabel('Longitudinal Velocity (m/s)')
legend('Planned','Simulated')
subplot(3,2,2)
plot(time,gamma0)
hold on
plot(time,gamma)
xlabel('Time (s)')
ylabel('Steering Angle')
ylim([-.8 .8])
legend('Planned','Simulated')
subplot(3,2,4)
plot(time,wxdes)
hold on
plot(time,Zn(:,5))
xlabel('Time (s)')
ylabel('Angular velocity rear wheel')
legend('Commanded','Simulated')
subplot(3,2,6)
plot(time,Fx)
hold on
plot(time,Fyf)
plot(time,Fyr)
xlabel('Time')
ylabel('Tire Forces (N)')
legend('Fx','Fyf','Fyr')

