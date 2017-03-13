function [dzdt,dfdz,dfdp] = lygerosparam(z,u,p)
%lygeros function with small angle approximations for tire forces, generic
%parameters

%states
X=z(1);
Y=z(2);
psi=z(3);%heading
vy=z(4);%lateral velocity
w=z(5);%yaw rate
%inputs
d=u(1); %steering input
vx=u(2); %longitudal velocity
%parameters
p1=p(1);
p2=p(2);
p3=p(3);
p4=p(4);
p5=p(5);
p6=p(6);
p7=p(7);
p8=p(8);
p9=p(9);
p10=p(10);


dzdt=[vx*cos(psi) - vy*sin(psi);...
                                                          vy*cos(psi) + vx*sin(psi);...
                                                                                  w;...
 (p2*vy)/vx - vx*w + (p1*w)/vx + d*p3*cos(d) - (p5*vy*cos(d))/vx - (p4*w*cos(d))/vx;...
       d*p6*cos(d) - (p9*w)/vx - (p10*vy)/vx - (p8*vy*cos(d))/vx - (p7*w*cos(d))/vx];

 dfdz=[ 0, 0, - vy*cos(psi) - vx*sin(psi),             -sin(psi),                           0;...
 0, 0,   vx*cos(psi) - vy*sin(psi),              cos(psi),                           0;...
 0, 0,                           0,                     0,                           1;...
 0, 0,                           0,   (p2 - p5*cos(d))/vx, -(vx^2 - p1 + p4*cos(d))/vx;...
 0, 0,                           0, -(p10 + p8*cos(d))/vx,        -(p9 + p7*cos(d))/vx];

dfdp=[    0,     0,        0,              0,               0,        0,              0,               0,     0,      0;...
    0,     0,        0,              0,               0,        0,              0,               0,     0,      0;...
    0,     0,        0,              0,               0,        0,              0,               0,     0,      0;...
 w/vx, vy/vx, d*cos(d), -(w*cos(d))/vx, -(vy*cos(d))/vx,        0,              0,               0,     0,      0;...
    0,     0,        0,              0,               0, d*cos(d), -(w*cos(d))/vx, -(vy*cos(d))/vx, -w/vx, -vy/vx];
 
 
end

