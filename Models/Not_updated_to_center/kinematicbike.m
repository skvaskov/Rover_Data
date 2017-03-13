function [dzdt,dfdz,dfdp] = kinematicbike(z,u,p)
%nonsingular kinematic bike model
%states
x=z(1);
y=z(2);
psi=z(3);
%parameters
lr=p(1); %length of center of mass from rear of axel
l=p(2); %length of wheelbase
p1=p(3); %scaling for steering angle input
p2=p(4); %shifting for steering angle
% p3=p(5); %"drift term"*vx
% p4=p(6); %constant "drift term"

%inputs
vx=u(2);
d=u(1);

dzdt=[ vx*(cos(psi) - (lr*tan(p2 + d*p1)*sin(psi))/l) - (vx*tan(p2 + d*p1)*sin(psi)*(l/2 - lr))/l;...
 vx*(sin(psi) + (lr*tan(p2 + d*p1)*cos(psi))/l) + (vx*tan(p2 + d*p1)*cos(psi)*(l/2 - lr))/l;...
                                                                      (vx*tan(p2 + d*p1))/l];
                      
dfdz=[ 0, 0, -(vx*(2*sin(psi) + tan(p2 + d*p1)*cos(psi)))/2;...
 0, 0,   vx*cos(psi) - (vx*tan(p2 + d*p1)*sin(psi))/2;...
 0, 0,                                              0];

dfdp=[ 0,                        0, -(d*vx*sin(psi))/(cos(2*p2 + 2*d*p1) + 1), -(vx*sin(psi))/(cos(2*p2 + 2*d*p1) + 1);...
 0,                        0,      (d*vx*cos(psi))/(2*cos(p2 + d*p1)^2),      (vx*cos(psi))/(2*cos(p2 + d*p1)^2);...
 0, -(vx*tan(p2 + d*p1))/l^2,           (d*vx*(tan(p2 + d*p1)^2 + 1))/l,           (vx*(tan(p2 + d*p1)^2 + 1))/l];
 
end

