function [dzdt dfdz dfdp] = wild(z,u,p)
% p=[m;p1;car;cr;cd];

x=z(1);%distance
vx=z(2); %velocity
V=u(1);%voltage

m=p(1);%mass
p1=p(2);%scaling factor for voltage
car=p(3);%tire stifness
cr=p(4);%rolling resistance
cd=p(5);%drag

 dzdt=[                                           vx;...
 -(cr + cd*vx^2 + (car*(vx - V*p1))/(vx))/m];

dfdz=[  0,                         1;...
0, -(2*cd*vx + car/(V*p1))/m];

dfdp=[                                             0,                   0,                     0,    0,       0;...
 (cr + cd*vx^2 + (car*(vx - V*p1))/(V*p1))/m^2, (car*vx)/(V*m*p1^2), -(vx - V*p1)/(V*m*p1), -1/m, -vx^2/m];


end

