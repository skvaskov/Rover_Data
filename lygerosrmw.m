function [dzdt,dfdz,dfdp]=lygerostan(z,u,p)
%states
X=z(1);
Y=z(2);
psi=z(3);%heading
vy=z(4);%lateral velocity
w=z(5);%yaw rate

m=p(1); %mass
Iz=p(2)*100; %moment of inertia
lf=p(3)/10; %lf (length of center of gravity from front wheel)
l=p(4)/10;  %wheelbase length
ca=p(5); %cornering stifness
pr=p(6);%scaling factor for rear tire stifness

%input
d=u(1); %steering input
vx=u(2); %longitudal velocity
dzdt= [vx*cos(psi) - vy*sin(psi);...
                                                              vy*cos(psi) + vx*sin(psi);...
                                                                                      w;...
             -(ca*pr*atan((vy - w*(l - lf))/vx) - ca*atan(d - (vy + lf*w)/vx)*cos(d))/m;...
 (ca*lf*atan(d - (vy + lf*w)/vx)*cos(d) + ca*pr*atan((vy - w*(l - lf))/vx)*(l - lf))/Iz];
  
dfdz= [ 0, 0, - vy*cos(psi) - vx*sin(psi),                                                                                                    -sin(psi),                                                                                                                 0;...
 0, 0,   vx*cos(psi) - vy*sin(psi),                                                                                                     cos(psi),                                                                                                                 0;...
 0, 0,                           0,                                                                                                            0,                                                                                                                 1;...
 0, 0,                           0,             -((ca*cos(d))/(vx*((d - (vy + lf*w)/vx)^2 + 1)) + (ca*pr)/(vx*((vy - w*(l - lf))^2/vx^2 + 1)))/m,       ((ca*pr*(l - lf))/(vx*((vy - w*(l - lf))^2/vx^2 + 1)) - (ca*lf*cos(d))/(vx*((d - (vy + lf*w)/vx)^2 + 1)))/m;...
 0, 0,                           0, ((ca*pr*(l - lf))/(vx*((vy - w*(l - lf))^2/vx^2 + 1)) - (ca*lf*cos(d))/(vx*((d - (vy + lf*w)/vx)^2 + 1)))/Iz, -((ca*lf^2*cos(d))/(vx*((d - (vy + lf*w)/vx)^2 + 1)) + (ca*pr*(l - lf)^2)/(vx*((vy - w*(l - lf))^2/vx^2 + 1)))/Iz];
 

dfdp=[                                                                           0,                                                                                         0,                                                                                                                                                                                         0,                                                                                              0,                                                                                0,                                           0;...
                                                                          0,                                                                                         0,                                                                                                                                                                                         0,                                                                                              0,                                                                                0,                                           0;...
                                                                           0,                                                                                         0,                                                                                                                                                                                         0,                                                                                              0,                                                                                0,                                           0;...
 (ca*pr*atan((vy - w*(l - lf))/vx) - ca*atan(d - (vy + lf*w)/vx)*cos(d))/m^2,                                                                                         0,                                                                                      -((ca*w*cos(d))/(vx*((d - (vy + lf*w)/vx)^2 + 1)) + (ca*pr*w)/(vx*((vy - w*(l - lf))^2/vx^2 + 1)))/m,                                                (ca*pr*w)/(m*vx*((vy - w*(l - lf))^2/vx^2 + 1)),              (atan(d - (vy + lf*w)/vx)*cos(d) - pr*atan((vy - w*(l - lf))/vx))/m,          -(ca*atan((vy - w*(l - lf))/vx))/m;...
                                                                           0, -(ca*lf*atan(d - (vy + lf*w)/vx)*cos(d) + ca*pr*atan((vy - w*(l - lf))/vx)*(l - lf))/Iz^2, -(ca*pr*atan((vy - w*(l - lf))/vx) - ca*atan(d - (vy + lf*w)/vx)*cos(d) + (ca*lf*w*cos(d))/(vx*((d - (vy + lf*w)/vx)^2 + 1)) - (ca*pr*w*(l - lf))/(vx*((vy - w*(l - lf))^2/vx^2 + 1)))/Iz, (ca*pr*atan((vy - w*(l - lf))/vx) - (ca*pr*w*(l - lf))/(vx*((vy - w*(l - lf))^2/vx^2 + 1)))/Iz, (lf*atan(d - (vy + lf*w)/vx)*cos(d) + pr*atan((vy - w*(l - lf))/vx)*(l - lf))/Iz, (ca*atan((vy - w*(l - lf))/vx)*(l - lf))/Iz];
 
end