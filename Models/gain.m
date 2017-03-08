function [dzdt,dfdz,dfdp] = gain(z,u,p)
x=z(1);
vx=z(2);
V=u(1);
tau=p(1);
k=p(2)/100;

dzdt=[                 vx;...
 (V*k)/tau- vx/tau];


dfdz=[ 0,      1;...
 0, -1/tau];

dfdp=[                0,     0;...
(vx - V*k)/tau^2, 1/100*V/tau];

end

