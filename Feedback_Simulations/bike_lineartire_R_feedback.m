function [dZdt] = bike_lineartire_R_feedback(t,Z,U,P)
%states
x=Z(1);
y=Z(2);
psi=Z(3);
vx=Z(4);
vy=Z(5);
w=Z(6);
xd=Z(7);
yd=Z(8);
psid=Z(9);


%input vector
gamma0=U(1);
vx0=U(2);

%parameter vector
%P=[m,Jg,l,d,ca, a, c,ky,kpsi,kx,kvx,kwx]
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


%error
ex=cos(psid)*(xd-x)+sin(psid)*(yd-y);
ey=-sin(psid)*(xd-x)+cos(psid)*(yd-y);
eh=psid-psi;
%input
gamma=ky*ey+kpsi*eh+gamma0;
%tire forces
if vx==0
    Fyf=0;
    Fyr=0;
else
Fyf=ca*(gamma-(vy+(l-d)*w)/vx);
Fyr=-ca*(vy-d*w)/vx;
%input
vxdes=kx*ex+kvx*(vx0-vx);
R=-a-c*vx^2+kvx*(vxdes-vx);


end
%dynamics
dZdt=[vx*cos(psi)-vy*sin(psi);...
    vx*sin(psi)+vy*cos(psi);...
    w;...
    1/m*(R-Fyf*sin(gamma))+vy*w;...
    1/m*(Fyr+Fyf*cos(gamma))-w*vx;...
    1/Jg*(-d*Fyr+(l-d)*Fyf*cos(gamma));...
    vx0/l*(l*cos(psid)-d*sin(psid)*tan(gamma0));...
    vx0/l*(l*sin(psid)+d*cos(psid)*tan(gamma0));...
    vx0/l*tan(gamma0)];

    


end

