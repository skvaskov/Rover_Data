function [dZdt] = bike_lineartire_R_feedback(t,Z,steerfun,vxfun,P)
%states
x=Z(1);
y=Z(2);
psi=Z(3);
vx=Z(4);
vy=Z(5);
w=Z(6);
gamma=Z(7);
xd=Z(8);
yd=Z(9);
psid=Z(10);


%input vector
gamma0=steerfun(t);
vx0=vxfun(t);

%parameter vector
%P=[m,Jg,l,d,ca, a, c,ky,kpsi,kx,kvx,kwx,kgd]
m=P(1);
Jg=P(2);
l=P(3);
d=P(4);
ca=P(5);
a=P(6);
b=P(7);
c=P(8);
ky=P(9);
kpsi=P(10);
kw=P(11);
kvy=P(12);
kx=P(13);
kvx=P(14);
kwx=P(15);
kgamma=P(16);

%error
%error
ex=cos(psid)*(xd-x)+sin(psid)*(yd-y);
ey=-sin(psid)*(xd-x)+cos(psid)*(yd-y);
eh=psid-psi;
evx=vx0-vx;
ew=vx0/l*tan(gamma0)-w;
evy=vx0*d/l*tan(gamma0)-vy;
%input
gammades=ky*ey+kpsi*eh+kw*ew+kvy*evy+gamma0;

%tire forces
if vx==0
    Fyf=0;
    Fyr=0;
else
Fyf=ca*(gamma-(vy+(l-d)*w)/vx);
Fyr=-ca*(vy-d*w)/vx;
%input
vxdes=kx*ex+kvx*evx+vx0;
R=-a-b*vx-c*vx^2+kwx*(vxdes-vx);


end
%dynamics
dZdt=[vx*cos(psi)-vy*sin(psi);...
    vx*sin(psi)+vy*cos(psi);...
    w;...
    1/m*(R-Fyf*sin(gamma))+vy*w;...
    1/m*(Fyr+Fyf*cos(gamma))-w*vx;...
    1/Jg*(-d*Fyr+(l-d)*Fyf*cos(gamma));
    kgd*(gammades-gamma);...
    vx0/l*(l*cos(psid)-d*sin(psid)*tan(gamma0));...
    vx0/l*(l*sin(psid)+d*cos(psid)*tan(gamma0));...
    vx0/l*tan(gamma0)];

    


end

