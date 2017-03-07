function [dzdt, dfdz, dfdp]=motor(z,u,p)
%parameters
m=p(1);
J=p(2); %moment of inertia of rotor kg.ms^2
b=p(3); %motor viscous friction constant N.m.s
ke=p(4); %electromotive force constant V/rad/sec
kt=p(5); %motor torque constant N.m/amp
R=p(6); %electric resistance ohm
L=p(7); %electric inductance H
r=p(8); % tire radius
gr=p(9);%gear ratio
cr=p(10);% rolling resistance
cd=p(11);% drag coefficent

%states
x=z(1);%distance
vx=z(2); %velocity (assuming rwos)
i=z(3); %current
%input
V=u(1); %voltage

dzdt=[                                          vx;...
 - cd*vx^2 - (b*vx)/J - cr/m + (gr*i*kt*r)/J;...
            V/L - (R*i)/L - (ke*vx)/(L*gr*r)];


dfdz=[  0,               1,           0;...
 0, - 2*cd*vx - b/J, (gr*kt*r)/J;...
 0,    -ke/(L*gr*r),        -R/L];

dfdp=[      0,                      0,     0,            0,          0,    0,                                      0,                  0,                  0,    0,     0;...
 cr/m^2, (b*vx - gr*i*kt*r)/J^2, -vx/J,            0, (gr*i*r)/J,    0,                                      0,        (gr*i*kt)/J,         (i*kt*r)/J, -1/m, -vx^2;...
      0,                      0,     0, -vx/(L*gr*r),          0, -i/L, (ke*vx - V*gr*r + R*gr*i*r)/(L^2*gr*r), (ke*vx)/(L*gr*r^2), (ke*vx)/(L*gr^2*r),    0,     0];
 



end