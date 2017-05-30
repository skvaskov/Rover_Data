%simulate trajectories for bike model with combined tire forces

clear all
close all
clc

%define parameters
m=2.759;
Jg=.1733;
l=.3;
d=.3-.1585;
avx=0;
bvx=0;
cvx=0;
Re=sqrt(.116^2-.01^2);
a=.01;
b=.07;
k=8.57142e5;
mu=0.6;
mu0=0.9;
ky=0;
kpsi=0;
kw=.5;
kvy=0;
kx=0;
kvx=20;
kwx=.85;
kgamma=10;
P=[m,Jg,l,d,avx, bvx, cvx,Re, a,b,k,mu,mu0,ky,kpsi,kw,kvy,kx,kvx,kwx,kgamma];

%enter desired steering and velocity inputs (for trajectory to follow)
num=10;
gmax=5; %maximum steering angle (degrees)
vmax=1;%maximum velocity (m/s)
steps=5*ones(1,num);
garray=rand(1,num)*gmax*2*pi/180-gmax*pi/180;
vxarray=rand(1,num)*vmax;
steerfun=@(t) step(t,garray,steps);
vxfun=@(t) step(t,vxarray,steps);

tarray=cumsum(steps);
tspan=[0,tarray(end)];
Z0=[0;0;0;vxarray(1);vxarray(1)/Re;0;0;garray(1)];
Z0=[Z0;Z0(1:3)];
[tconst,zconst]=ode45(@(t,Z) bikeconstv(t,Z,steerfun,vxfun,[l;d]),tspan,Z0(1:3));
[tfb,zfb]=ode45(@(t,Z) bike_paratire_wx_feedback(t,Z,steerfun,vxfun,P),tspan,Z0);

%calculate commanded inputs, tire forces
len=length(zfb(:,1));
Fx=zeros(len,1);
Fyr=zeros(len,1);
Fyf=zeros(len,1);
gammades=zeros(len,1);
gamma0=zeros(len,1);
vx0=zeros(len,1);
wxdes=zeros(len,1);
tar=zeros(len,1);
taf=zeros(len,1);
Mr=zeros(len,1);
Mf=zeros(len,1);
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
%input vector
gamma0(i)=steerfun(t);
vx0(i)=vxfun(t);
%error
ex=cos(psid)*(xd-x)+sin(psid)*(yd-y);
ey=-sin(psid)*(xd-x)+cos(psid)*(yd-y);
eh=psid-psi;
evx=vx0(i)-vx;
ew=vx0(i)/l*tan(gamma0(i))-w;
evy=vx0(i)*d/l*tan(gamma0(i))-vy;
%input
gammades(i)=ky*ey+kpsi*eh+kw*ew+kvy*evy+gamma0(i);
gammadot=kgamma*(gammades(i)-gamma);
wxdes(i)=kx*ex+kvx*evx+vx0(i)/Re;

%slip angles
if vx==0
    tar(i)=0;
    taf(i)=0;
else
tar(i)=-(vy-(d-a)*w)/vx;
taf(i)=-(-vx*sin(gamma)+(vy+(l-d)*w)*cos(gamma)+a*(w+gammadot))/(vx*cos(gamma)+(vy+(l-d)*w)*sin(gamma));
end
%slip ratios
if wxdes(i)>=wx
    sigmax=(Re*wx-vx)/Re*wx;
else
    sigmax=(Re*wx-vx)/vx;
end
sigmayr=vx/(Re*wx)*tar(i);
sigmar=sqrt(sigmax^2+sigmayr^2);
sigmayf=taf(i);
sigmaf=sqrt(sigmayf^2);

%normal Forces (BIG ASSUMPTION that they are constant)
g=9.80655;
Fzf=m*g*d/l;
Fzr=m*g*(l-d)/l;
%limits for sliding
thetaf=4*a^2*b*k/(3*mu0*Fzf);
thetar=4*a^2*b*k/(3*mu0*Fzr);

%Lateral force and moment for front
if 0<sigmaf<=1/thetaf
    Fyf(i)=sigmayf/sigmaf*mu*Fzf*(3*thetaf*sigmaf-1/3*(3*thetaf*sigmaf)^2+1/27*(3*thetaf*sigmaf)^3);
    Mf(i)=mu*Fzf*sigmayf/sigmaf*a*(thetaf*sigmaf-3*thetaf^2*sigmaf^2+3*thetaf^3*sigmaf^3-thetaf^4*sigmaf^4);
elseif sigmaf==0
    Fyf(i)=0;
    Mf(i)=0;
else
    Fyf(i)=mu*Fzf;
    Mf(i)=0;
end

%Rear forces
if 0<sigmar&&sigmar<=1/thetar
Fyr(i)=sigmayr/sigmar*mu*Fzr*(3*thetar*sigmar-1/3*(3*thetar*sigmar)^2+1/27*(3*thetar*sigmar)^3);
Fx(i)=sigmax/sigmar*mu*Fzr*(3*thetar*sigmar-1/3*(3*thetar*sigmar)^2+1/27*(3*thetar*sigmar)^3);
Mr(i)=mu*Fzr*a*sigmayr/sigmar*(thetar*sigmar-3*thetar^2*sigmar^2+3*thetar^3*sigmar^3-thetar^4*sigmar^4);
elseif sigmar==0
    Fx(i)=0;
    Fyr(i)=0;
    Mr(i)=0;
else
    Fyr(i)=sigmayr/sigmar*mu*Fzr;
    Fx(i)=sigmax/sigmar*mu*Fzr;
    Mr(i)=0;
end

end

figure
subplot(3,2,1)
plot(zfb(:,9),zfb(:,10))
hold on
plot(zfb(:,1),zfb(:,2))
xlabel('X')
ylabel('Y')
legend('Plan (0)','Simulated')
subplot(3,2,3)
plot(tfb,zfb(:,11))
hold on
plot(tfb,zfb(:,3))
xlabel('Time (s)')
ylabel('Heading (rad)')
legend('Plan (0)','Simulated')
subplot(3,2,5)
plot(tfb,vx0)
hold on
plot(tfb,zfb(:,4))
xlabel('Time (s)')
ylabel('Longitudinal Velocity (m/s)')
legend('Plan (0)','Simulated')
subplot(3,2,2)
plot(tfb,gamma0)
hold on
plot(tfb,gammades)
plot(tfb,zfb(:,8))
xlabel('Time (s)')
ylabel('Steering Angle')
legend('Plan(0)','Command','Simulated')
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
 plot(tfb,Mf)
plot(tfb,Mr)
 xlabel('Time')
 ylabel('Tire Forces (N)')
 legend('Fx','Fyf','Fyr','Mf','Mr')

