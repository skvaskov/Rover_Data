function [dfdt,dfdZ,dfdP] = bike_lineartire_gaintuning(Z,U,P)
%car parameters
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
kwx=0.85;
kgamma=10;

%tuning parameters
ky=P(1);
kpsi=P(2);
kx=P(3);
kvx=P(4);


%states
x=Z(1);
y=Z(2);
psi=Z(3);
vx=Z(4);
wx=Z(5);
vy=Z(6);
w=Z(7);
gamma=Z(8);

%input vector U=g0,vx0, xd,yd,psid
gamma0=U(1);
vx0=U(2);
xd=U(3);
yd=U(4);
psid=U(5);

%error
ex=cos(psid)*(xd-x)+sin(psid)*(yd-y);
ey=-sin(psid)*(xd-x)+cos(psid)*(yd-y);
eh=psid-psi;
evx=vx0-vx;
%input
gammades=ky*ey+kpsi*eh+gamma0;

wxdes=kx*ex+kvx*evx+vx0/Re;

%tire forces
if vx==0 || wx==0
    Fyf=0;
    Fyr=0;
    Fx=0;
else
Fyf=ca*(gamma-(vy+(l-d)*w)/vx);
Fyr=-ca*(vy-d*w)/vx;
Fx=ca*(Re*wx-vx)/Re*wx;
end
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
dfdZ=[                   0,                    0, - vy*cos(psi) - vx*sin(psi),                                                                     cos(psi),                           0,                                   -sin(psi),                                                0,                                                                                 0;...
                   0,                    0,   vx*cos(psi) - vy*sin(psi),                                                                     sin(psi),                           0,                                    cos(psi),                                                0,                                                                                 0;...
                   0,                    0,                           0,                                                                            0,                           0,                                           0,                                                1,                                                                                 0;...
                   0,                    0,                           0,         -(b + 2*c*vx + (ca*wx)/Re + (ca*sin(gamma)*(vy - w*(d - l)))/vx^2)/m, -(ca*(vx - 2*Re*wx))/(Re*m),                  w + (ca*sin(gamma))/(m*vx),              vy - (ca*sin(gamma)*(d - l))/(m*vx),                  -(ca*sin(gamma) + ca*cos(gamma)*(gamma - (vy - w*(d - l))/vx))/m;...
   -kwx*kx*cos(psid),    -kwx*kx*sin(psid),                           0,                                                                     -kvx*kwx,                        -kwx,                                           0,                                                0,                                                                                 0;...
                   0,                    0,                           0,         ((ca*(vy - d*w))/vx^2 + (ca*cos(gamma)*(vy - w*(d - l)))/vx^2)/m - w,                           0,               -(ca*(cos(gamma) + 1))/(m*vx),  ((ca*d)/vx + (ca*cos(gamma)*(d - l))/vx)/m - vx,                   (ca*cos(gamma) - ca*sin(gamma)*(gamma - (vy - w*(d - l))/vx))/m;...
                   0,                    0,                           0, -((ca*d*(vy - d*w))/vx^2 + (ca*cos(gamma)*(vy - w*(d - l))*(d - l))/vx^2)/Jg,                           0, ((ca*d)/vx + (ca*cos(gamma)*(d - l))/vx)/Jg, -((ca*d^2)/vx + (ca*cos(gamma)*(d - l)^2)/vx)/Jg, -(ca*cos(gamma)*(d - l) - ca*sin(gamma)*(gamma - (vy - w*(d - l))/vx)*(d - l))/Jg;...
 kgamma*ky*sin(psid), -kgamma*ky*cos(psid),                -kgamma*kpsi,                                                                            0,                           0,                                           0,                                                0,                                                                           -kgamma];
 
dfdP = [                                                 0,                   0,                                              0,               0;...
                                                 0,                   0,                                              0,               0;...
                                                 0,                   0,                                              0,               0;...
                                                 0,                   0,                                              0,               0;...
                                                 0,                   0, -kwx*(cos(psid)*(x - xd) + sin(psid)*(y - yd)), -kwx*(vx - vx0);...
                                                 0,                   0,                                              0,               0;...
                                                 0,                   0,                                              0,               0;...
 -kgamma*(cos(psid)*(y - yd) - sin(psid)*(x - xd)), kgamma*(psid - psi),                                              0,               0];
 
end

