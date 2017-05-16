clear all
close all
clc
%symoblically find derivatives for gain tuning
syms x y psi vx wx vy w m Jg l d ca Re a b c ky kpsi kx kvx kwx gamma gamma0 vx0 xd yd psid kgamma 
%define states
Z=[x,y,psi,vx,wx,vy,w,gamma];
P=[ky,kpsi,kx,kvx];
%error
ex=cos(psid)*(xd-x)+sin(psid)*(yd-y);
ey=-sin(psid)*(xd-x)+cos(psid)*(yd-y);
eh=psid-psi;
evx=vx0-vx;
%input
gammades=ky*ey+kpsi*eh+gamma0;
wxdes=kx*ex+kvx*evx+vx0/Re;

%tire forces
Fyf=ca*(gamma-(vy+(l-d)*w)/vx);
Fyr=-ca*(vy-d*w)/vx;
Fx=ca*(Re*wx-vx)/Re*wx;
R=-a-b*vx-c*vx^2+Fx;

%dynamics
dfdt=[vx*cos(psi)-vy*sin(psi);...
    vx*sin(psi)+vy*cos(psi);...
    w;...
    1/m*(R-Fyf*sin(gamma))+vy*w;
    kwx*(wxdes-wx);...
    1/m*(Fyr+Fyf*cos(gamma))-w*vx;...
    1/Jg*(-d*Fyr+(l-d)*Fyf*cos(gamma));...
    kgamma*(gammades-gamma)];

%gradients
dfdZ=simplify(jacobian(dfdt,Z))
dfdP=simplify(jacobian(dfdt,P))