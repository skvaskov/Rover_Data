function [dZdt] = bike_lineartire_wx_feedback(t,Z,steerfun,vxfun,P)
%states
x=Z(1);
y=Z(2);
psi=Z(3);
vx=Z(4);
wx=Z(5);
vy=Z(6);
w=Z(7);
gamma=Z(8);
xd=Z(9);
yd=Z(10);
psid=Z(11);

%input vector
gamma0=steerfun(t);
vx0=vxfun(t);

%parameter vector
%P=[m,Jg,l,d,ca, Re, a,b, c,ky,kpsi,kw,kx,kvx,kwx,kgamma]
m=P(1);
Jg=P(2);
l=P(3);
d=P(4);
ca=P(5);
Re=P(6);
a=P(7);
b=P(8);
c=P(9);
ky=P(10);
kpsi=P(11);
kx=P(12);
kvx=P(13);
kwx=P(14);
kgamma=P(15);

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
dZdt=[vx*cos(psi)-vy*sin(psi);...
    vx*sin(psi)+vy*cos(psi);...
    w;...
    1/m*(R-Fyf*sin(gamma))+vy*w;
    kwx*(wxdes-wx);...
    1/m*(Fyr+Fyf*cos(gamma))-w*vx;...
    1/Jg*(-d*Fyr+(l-d)*Fyf*cos(gamma));...
    kgamma*(gammades-gamma);...
    vx0/l*(l*cos(psid)-d*sin(psid)*tan(gamma0));...
    vx0/l*(l*sin(psid)+d*cos(psid)*tan(gamma0));...
    vx0/l*tan(gamma0)];

    


end

