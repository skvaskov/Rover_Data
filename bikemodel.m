function [dzdt dfdz dfdp] = bikemodel(z,u,p)
%states
X=z(1);
Y=z(2);
H=z(3);
VX=z(4);

%inputs
u1=u(1);%steering (please scale down to <1)
u2=u(2);%throttle (please scale down to <1)

%p = [m  lf l ca cm1 cm2 cd cr p1 p2]' ;
m=p(1);%mass
lf=p(2);%length of center of mass from front wheel
l=p(3);%wheelbase
ca=p(4);%cornering stifness
cm1=p(5);%something for motor
cm2=p(6);%something for motor
cd=p(7);%drag
cr=(8);%rolling resistance
p1=(9);%scaling for steering
p2=(10);%scaling for throttle

dzdt=[VX*cos(H + atan((tan(p1*u1)*(l - lf))/l))*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2);...
                                                                            VX*sin(H + atan((tan(p1*u1)*(l - lf))/l))*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2);...
                                                                                                                                              (VX*tan(p1*u1))/l;...
 -(cr + VX^2*cd + ca*sin(p1*u1)*(p1*u1 - ((VX*lf*tan(p1*u1))/l + (VX*tan(p1*u1)*(l - lf))/l)/VX) - p2*u2*(cm1 - VX*cm2) - (VX^2*m*tan(p1*u1)^2*(l - lf))/l^2)/m];
 
dfdz=[ 0, 0, -VX*sin(H + atan((tan(p1*u1)*(l - lf))/l))*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2),                                               cos(H + atan((tan(p1*u1)*(l - lf))/l))*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2);...
 0, 0,  VX*cos(H + atan((tan(p1*u1)*(l - lf))/l))*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2),                                               sin(H + atan((tan(p1*u1)*(l - lf))/l))*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2);...
 0, 0,                                                                                    0,                                                                                                                   tan(p1*u1)/l;...
 0, 0,                                                                                    0, -(2*VX*lf*m*sin(p1*u1)^2 - 2*VX*l*m*sin(p1*u1)^2 + 2*VX*cd*l^2*cos(p1*u1)^2 + cm2*l^2*p2*u2*cos(p1*u1)^2)/(l^2*m*cos(p1*u1)^2)];
 
dfdp= [                                                                                                                                                    0,   (VX*tan(p1*u1)*sin(H + atan((tan(p1*u1)*(l - lf))/l)))/(l*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)) - (VX*tan(p1*u1)^2*cos(H + atan((tan(p1*u1)*(l - lf))/l))*(2*l - 2*lf))/(2*l^2*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)), (VX*lf*tan(p1*u1)^2*cos(H + atan((tan(p1*u1)*(l - lf))/l))*(l - lf))/(l^3*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)) - (VX*lf*tan(p1*u1)*sin(H + atan((tan(p1*u1)*(l - lf))/l)))/(l^2*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)),                                                           0,         0,             0,       0,    0, (VX*u1*tan(p1*u1)*cos(H + atan((tan(p1*u1)*(l - lf))/l))*(tan(p1*u1)^2 + 1)*(l - lf)^2)/(l^2*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)) - (VX*u1*sin(H + atan((tan(p1*u1)*(l - lf))/l))*(tan(p1*u1)^2 + 1)*(l - lf))/(l*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)),                     0;...
                                                                                                                                                    0, - (VX*tan(p1*u1)*cos(H + atan((tan(p1*u1)*(l - lf))/l)))/(l*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)) - (VX*tan(p1*u1)^2*sin(H + atan((tan(p1*u1)*(l - lf))/l))*(2*l - 2*lf))/(2*l^2*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)), (VX*lf*tan(p1*u1)*cos(H + atan((tan(p1*u1)*(l - lf))/l)))/(l^2*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)) + (VX*lf*tan(p1*u1)^2*sin(H + atan((tan(p1*u1)*(l - lf))/l))*(l - lf))/(l^3*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)),                                                           0,         0,             0,       0,    0, (VX*u1*cos(H + atan((tan(p1*u1)*(l - lf))/l))*(tan(p1*u1)^2 + 1)*(l - lf))/(l*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)) + (VX*u1*tan(p1*u1)*sin(H + atan((tan(p1*u1)*(l - lf))/l))*(tan(p1*u1)^2 + 1)*(l - lf)^2)/(l^2*((tan(p1*u1)^2*(l - lf)^2)/l^2 + 1)^(1/2)),                     0;...
                                                                                                                                                    0,                                                                                                                                                                                                                                0,                                                                                                                                                                                                             -(VX*tan(p1*u1))/l^2,                                                           0,         0,             0,       0,    0,                                                                                                                                                                                                                                       (VX*u1*(tan(p1*u1)^2 + 1))/l,                     0;...
 (cr*cos(p1*u1) - ca*sin(p1*u1)^2 + VX^2*cd*cos(p1*u1) - cm1*p2*u2*cos(p1*u1) + (ca*p1*u1*sin(2*p1*u1))/2 + VX*cm2*p2*u2*cos(p1*u1))/(m^2*cos(p1*u1)),                                                                                                                                                                                                         -(VX^2*tan(p1*u1)^2)/l^2,                                                                                                                                                                          (VX^2*sin(p1*u1)^2*(l - 2*lf))/(l^3*(sin(p1*u1)^2 - 1)), (sin(p1*u1)^2 - p1*u1*cos(p1*u1)*sin(p1*u1))/(m*cos(p1*u1)), (p2*u2)/m, -(VX*p2*u2)/m, -VX^2/m, -1/m,                                                                                                                              (u1*(2*VX^2*l*m*sin(p1*u1) - 2*VX^2*lf*m*sin(p1*u1) + ca*l^2*cos(p1*u1)*sin(p1*u1) - ca*l^2*p1*u1*cos(p1*u1)^4))/(l^2*m*cos(p1*u1)^3), (u2*(cm1 - VX*cm2))/m];
 
end

