function [dzdt,dfdz,dfdp]=lygerosraj(z,u,p)
%states
x=z(1);
y=z(2);
psi=z(3);%heading
vy=z(4);%lateral velocity
w=z(5);%yaw rate

m=p(1); %mass
Iz=p(2); %moment of inertia
lf=p(3); %lf (length of center of gravity from front wheel)
l=p(4);  %wheelbase length
ca=p(5); %cornering stifness
pr=p(6);%scaling factor for rear tire stifness

%input
d=u(1); %steering input
vx=u(2); %longitudal velocity

dzdt=[ vx*cos(psi) - vy*sin(psi);...
                                                      vy*cos(psi) + vx*sin(psi);...
                                                                              w;...
    -(m*vx*w - ca*cos(d)*(d - (vy + lf*w)/vx) + (ca*pr*(vy - w*(l - lf)))/vx)/m;...
 (ca*lf*cos(d)*(d - (vy + lf*w)/vx) + (ca*pr*(vy - w*(l - lf))*(l - lf))/vx)/Iz];
  
dfdz=[ 0, 0, - vy*cos(psi) - vx*sin(psi),                                -sin(psi),                                                   0;...
 0, 0,   vx*cos(psi) - vy*sin(psi),                                 cos(psi),                                                   0;...
 0, 0,                           0,                                        0,                                                   1;...
 0, 0,                           0,               -(ca*(pr + cos(d)))/(m*vx), -(m*vx - (ca*pr*(l - lf))/vx + (ca*lf*cos(d))/vx)/m;...
 0, 0,                           0, -(ca*(lf*pr - l*pr + lf*cos(d)))/(Iz*vx),   -((ca*lf^2*cos(d))/vx + (ca*pr*(l - lf)^2)/vx)/Iz];
 
dfdp= [                                                                                       0,                                                                                 0,                                                                                      0,                                     0,                                                                        0,                                       0;...
                                                                                       0,                                                                                 0,                                                                                      0,                                     0,                                                                        0,                                       0;...
                                                                                       0,                                                                                 0,                                                                                      0,                                     0,                                                                        0,                                       0;...
 (m*vx*w - ca*cos(d)*(d - (vy + lf*w)/vx) + (ca*pr*(vy - w*(l - lf)))/vx)/m^2 - (vx*w)/m,                                                                                 0,                                                           -(ca*w*(pr + cos(d)))/(m*vx),                      (ca*pr*w)/(m*vx),              (cos(d)*(d - (vy + lf*w)/vx) - (pr*(vy - w*(l - lf)))/vx)/m,          -(ca*(vy - w*(l - lf)))/(m*vx);...
                                                                                       0, -(ca*lf*cos(d)*(d - (vy + lf*w)/vx) + (ca*pr*(vy - w*(l - lf))*(l - lf))/vx)/Iz^2, -(ca*(pr*vy + vy*cos(d) - d*vx*cos(d) + 2*lf*w*cos(d) - 2*l*pr*w + 2*lf*pr*w))/(Iz*vx), (ca*pr*(vy - 2*l*w + 2*lf*w))/(Iz*vx), (lf*cos(d)*(d - (vy + lf*w)/vx) + (pr*(vy - w*(l - lf))*(l - lf))/vx)/Iz, (ca*(vy - w*(l - lf))*(l - lf))/(Iz*vx)];
 
end