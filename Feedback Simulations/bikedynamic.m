function [dZdt] = bikedynamic(Z,t,U,P)
%bicycle model with lateral tire forces
%Parameter Vector P=[m,Jg,l,d,a,c,R,a]
m=P(1);
Jg=P(2);
l=P(3);
d=P(4);
a=P(5);
c=P(6);
Re=sqrt(P(7)^2-P(8)^2);
%state vector Z=[x,y,psi,vx,vy,w]
x=Z(1);
y=Z(2);
psi=Z(3);
vx=Z(4);
vy=Z(5);
w=Z(6);
%input vector U=[gamma,gammadot,wx]
gamma=U(1);
gammadot=U(2);
wx=U(3);
%rear slip angle
tar=

%longitudinal rear tire force
s=
Fx=
R=-a-c*vx^2+Fx;

dZdt=[vx*cos(psi)-vy*sin(psi);...
    vx*sin(psi)+vy*cos(psi);...
    w;...
    1/m*(R-Ff*sin(gamma))+vy*w;...
    1/m*(Fr+Ff*cos(gamma))-w*vx;...
    1/Jg*(-d*Fr+(l-d)*Ff*cos(gamma)+Mr+Mf)];
    




end

