function [dzdt,dfdz,dfdp] = kinematicbike(z,u,p)
%nonsingular kinematic bike model
%states
x=z(1);
y=z(2);
psi=z(3);
%parameters
 scale=[.1,.001,.1];
l=p(1)*scale(1); %length of wheelbase
p1=p(2)*scale(2); %scaling for steering angle input
p2=p(3)*scale(3); %shifting for steering angle
% p3=p(5); %"drift term"*vx
% p4=p(6); %constant "drift term"

%inputs
vx=u(2);
d=u(1);

dzdt=[ vx*(cos(psi) - (tan(p2 + d*p1)*sin(psi))/2);...
 vx*(sin(psi) + (tan(p2 + d*p1)*cos(psi))/2);...
                       (vx*tan(p2 + d*p1))/l];
                      
dfdz=[ 0, 0, -vx*(sin(psi) + (tan(p2 + d*p1)*cos(psi))/2);...
 0, 0,  vx*(cos(psi) - (tan(p2 + d*p1)*sin(psi))/2);...
 0, 0,                                            0];

dfdp=[                        0, -(d*vx*sin(psi)*(tan(p2 + d*p1)^2 + 1))/2, -(vx*sin(psi)*(tan(p2 + d*p1)^2 + 1))/2;...
                        0,  (d*vx*cos(psi)*(tan(p2 + d*p1)^2 + 1))/2,  (vx*cos(psi)*(tan(p2 + d*p1)^2 + 1))/2;...
 -(vx*tan(p2 + d*p1))/l^2,           (d*vx*(tan(p2 + d*p1)^2 + 1))/l,           (vx*(tan(p2 + d*p1)^2 + 1))/l];
 
%add scaling to jacobian for parameters 
 
  scale=repmat(scale,3,1);
  dfdp=scale.*dfdp;

end

