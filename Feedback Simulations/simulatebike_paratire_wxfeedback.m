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
ky=1;
kpsi=1;
kx=1;
kvx=.5;
kwx=.85;
P=[m,Jg,l,d, avx, bvx, cvx, Re, a,b,k,mu,mu0,ky,kpsi,kx,kvx,kwx];

%center desired steering and velocity (trajectory paramters)
gamma0=2*pi/180;
gamma0dot=0;
vx0=.5;

tspan=[0,10];
Z0=[0;0;0;.1;.1/Re;0;0];
Z0=[Z0;Z0(1:3)];
[tconst,zconst]=ode45(@(t,Z) bikeconstv(t,Z,[gamma0;vx0],[l;d]),tspan,Z0(1:3));
[tfb,zfb]=ode45(@(t,Z) bike_paratire_wx_feedback(t,Z,[gamma0;gamma0dot;vx0],P),tspan,Z0);

%calculate commanded inputs, tire forces
l=length(zfb(:,1));
Fx=zeros(l,1);
Fyr=zeros(l,1);
Fyf=zeros(l,1);
gamma=zeros(l,1);
gammadot=zeros(l,1);
wxdes=zeros(l,1);
tar=zeros(l,1);
taf=zeros(l,1);
Mr=zeros(l,1);
Mf=zeros(l,1);
for i=1:l
x=zfb(i,1);
y=zfb(i,2);
psi=zfb(i,3);
vx=zfb(i,4);
wx=zfb(i,5);
vy=zfb(i,6);
w=zfb(i,7);
xd=zfb(i,8);
yd=zfb(i,9);
psid=zfb(i,10);
%error
ex=cos(psid)*(xd-x)+sin(psid)*(yd-y);
ey=-sin(psid)*(xd-x)+cos(psid)*(yd-y);
eydot=(d-cos(psid)*(xd-x)-sin(psid)*(yd-y))*vx0/l*tan(gamma0)+...
    vx*sin(psid-psi)+vy*cos(psid-psi);


eh=psid-psi;
ehdot=vx0/l*tan(gamma0)-w;
evx=vx0-vx;
%input
gamma(i)=ky*ey+kpsi*eh+gamma0;
gammadot(i)=ky*eydot+kpsi*ehdot+gamma0dot;
wxdes(i)=kx*ex+kvx*evx+vx0/Re;

%slip angles
if vx==0
    tar(i)=0;
    taf(i)=0;
else
tar(i)=-(vy-(d-a)*w)/vx;
taf(i)=-(-vx*sin(gamma(i))+(vy+(l-d)*w)*cos(gamma(i))+a*(w+gammadot(i)))/(vx*cos(gamma(i))+(vy+(l-d)*w)*sin(gamma(i)));
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

