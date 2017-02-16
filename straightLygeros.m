function [dzdt, dfdz, dfdp] = straightLygeros(z,u,p)
%guess for imu time [2.75970884532955;11.8320542595609;2.59043534446374;0;10.7061070245412;10.0049883987681]
X=z(1); %distance
VX=z(2); %velocity
%p = [m cm1 cm2 cd cr p2]' ;
u2=u(1); %throttle 
%parameters
m=p(1);
cm1=p(2);
cm2=p(3);
cd=p(4);
cr=p(5);
p2=p(6);

dzdt=[   VX;...
 -(cr + VX^2*cd - p2*u2*(cm1 - VX*cm2))/m];

dfdz=[ 0,                        1;...
 0, -(2*VX*cd + cm2*p2*u2)/m];

 dfdp=[                                         0,         0,             0,       0,    0,                     0;...
 (cr + VX^2*cd - p2*u2*(cm1 - VX*cm2))/m^2, (p2*u2)/m, -(VX*p2*u2)/m, -VX^2/m, -1/m, (u2*(cm1 - VX*cm2))/m];
 
    
end