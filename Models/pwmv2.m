function [dzdt,dfdz,dfdp] = pwmv2(z,u,p)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
vx=z;
u1=u;
c1=p(1);
c2=p(2);
c3=p(3);
c4=p(4);
c5=p(5);
c6=p(6);
c7=p(7);


dzdt=c7*u1^2*vx + c6*u1^2 + c4*u1*vx + c2*u1 + c5*vx^2 + c3*vx + c1;
dfdz=c7*u1^2 + c4*u1 + c3 + 2*c5*vx;
dfdp=[ 1, u1, vx, u1*vx, vx^2, u1^2, u1^2*vx];

end

