function [dzdt,dfdz,dfdp] = lygerosStraight(z,u,p)
%protion of model presented in lygeros paper exclusively with straight line
x=z(1);%distance
vx=z(2);%velocity
m=p(1);%mass 
cm1=p(2);%motor constant 1
cm2=p(3);%motor constant 2
cr=p(4)/100;%rolling resistance
cd=p(5)/10000;%drag coefficient
d=u(1);%duty cycle
dzdt=[vx;...
 (d*(cm1 - cm2*vx))/m - cd*vx^2 - cr];

dfdz=[ 0,                     1;...
 0, - 2*cd*vx - (cm2*d)/m];

dfdp=[                       0,   0,         0,  0,     0;...
 -(d*(cm1 - cm2*vx))/m^2, d/m, -(d*vx)/m, -1, -vx^2];
end

