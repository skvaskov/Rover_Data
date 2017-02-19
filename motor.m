function [dzdt, dfdz, dfdp]=motor(z,u,p)
%parameters
J=p(1); %moment of inertia of rotor kg.ms^2
b=p(2); %motor viscous friction constant N.m.s
ke=p(3); %electromotive force constant V/rad/sec
kt=p(4); %motor torque constant N.m/amp
R=p(5); %electric resistance ohm
L=p(6); %electric inductance H
r=p(7); % tire radius
cr=p(8);% rolling resistance
p2=p(9);% scaling for input

%states
v=z(1); %velocity (assuming rwos)
ic=z(2); %current
%input
Volt=u(1); %voltage

dzdt=[          -(J*cr + b*v - ic*kt*r)/J;...
 -(ke*v + R*ic*r - Volt*p2*r)/(L*r)];

dfdz=[-b/J, (kt*r)/J;...
-ke/(L*r),     -R/L];

dfdp=[ (b*v - ic*kt*r)/J^2, -v/J,        0, (ic*r)/J,     0,                                   0,      (ic*kt)/J, -1,      0;...
                0,    0, -v/(L*r),        0, -ic/L, (ke*v + R*ic*r - Volt*p2*r)/(L^2*r), (ke*v)/(L*r^2),  0, Volt/L];
 




end