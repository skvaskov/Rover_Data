function [dzdt,dfdz,dfdp] = picruise(z,u,p)
%linear function for PI cruise control 
%states
x=z(1);
vx=z(2);
e=z(3);
%input
u2=u;

%parameters
a=p(1);
b=p(2);
c=p(3);
kp=p(4);
ki=p(5);
ps=p(6); %scaling for input

dzdt=[                                   vx;...
 ki*e - b*vx - a - c*vx^2 - kp*(vx - ps*u2);...
                                 ps*u2 - vx];
                             
dfdz=[ 0,                 1,  0;...
 0, - b - kp - 2*c*vx, ki;...
 0,                -1,  0];
 

dfdp=[  0,   0,     0,          0, 0,     0;...
 -1, -vx, -vx^2, ps*u2 - vx, e, kp*u2;...
  0,   0,     0,          0, 0,    u2];
end

