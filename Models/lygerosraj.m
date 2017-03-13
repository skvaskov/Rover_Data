function [dzdt,dfdz,dfdp]=lygerosraj(z,u,p)
%dyanmic bike model from lygeros paper with small angle approximations for
%tire force angles

%states
x=z(1);
y=z(2);
psi=z(3);%heading
vy=z(4);%lateral velocity
w=z(5);%yaw rate

m=p(1); %mass
Iz=p(2)/10; %moment of inertia
lf=p(3)/10; %lf (length of center of gravity from front wheel)
l=p(4)/10;  %wheelbase length
ca=p(5)*10; %cornering stifness
pr=p(6);%scaling factor for rear tire stifness
p1=p(7);%scaling for steering input
p2=p(8);%shift for steeringinput

%input
d=u(1); %steering input
vx=u(2); %longitudal velocity

dzdt=[vx*cos(psi) - vy*sin(psi)
                                                                                                                vy*cos(psi) + vx*sin(psi);...
                                                                                                                                        w;...
 ca*cos(p2 + d*p1)*(d - (vy + (l*w)/2)/vx)*(1/m + (lf*(l/2 - lf))/Iz) - vx*w - (ca*pr*(vy - (l*w)/2)*(1/m - ((l - lf)*(l/2 - lf))/Iz))/vx;...
                                                   (ca*lf*cos(p2 + d*p1)*(d - (vy + (l*w)/2)/vx) + (ca*pr*(vy - (l*w)/2)*(l - lf))/vx)/Iz];
  
dfdz=[ 0, 0, - vy*cos(psi) - vx*sin(psi),                                -sin(psi),                                                   0;...
 0, 0,   vx*cos(psi) - vy*sin(psi),                                 cos(psi),                                                   0;...
 0, 0,                           0,                                        0,                                                   1;...
 0, 0,                           0,               -(ca*(pr + cos(d)))/(m*vx), -(m*vx - (ca*pr*(l - lf))/vx + (ca*lf*cos(d))/vx)/m;...
 0, 0,                           0, -(ca*(lf*pr - l*pr + lf*cos(d)))/(Iz*vx),   -((ca*lf^2*cos(d))/vx + (ca*pr*(l - lf)^2)/vx)/Iz];
 
dfdp= [                                                                                 0,                                                                                                                       0,                                                                                                              0,                                                                                                                                                                                                                                 0,                                                                                                                           0,                                                        0,                                                                       0,                                                                     0;...
                                                                                 0,                                                                                                                       0,                                                                                                              0,                                                                                                                                                                                                                                 0,                                                                                                                           0,                                                        0,                                                                       0,                                                                     0;...
                                                                                 0,                                                                                                                       0,                                                                                                              0,                                                                                                                                                                                                                                 0,                                                                                                                           0,                                                        0,                                                                       0,                                                                     0;...
 (ca*pr*(vy - (l*w)/2))/(m^2*vx) - (ca*cos(p2 + d*p1)*(d - (vy + (l*w)/2)/vx))/m^2, - (ca*lf*cos(p2 + d*p1)*(d - (vy + (l*w)/2)/vx)*(l/2 - lf))/Iz^2 - (ca*pr*(vy - (l*w)/2)*(l - lf)*(l/2 - lf))/(Iz^2*vx), - (ca*pr*(2*vy - l*w)*(3*l - 4*lf))/(4*Iz*vx) - (ca*cos(p2 + d*p1)*(l - 4*lf)*(2*vy - 2*d*vx + l*w))/(4*Iz*vx), (ca*pr*(vy - (l*w)/2)*((l - lf)/(2*Iz) + (l/2 - lf)/Iz))/vx - (ca*w*cos(p2 + d*p1)*(1/m + (lf*(l/2 - lf))/Iz))/(2*vx) + (ca*pr*w*(1/m - ((l - lf)*(l/2 - lf))/Iz))/(2*vx) + (ca*lf*cos(p2 + d*p1)*(d - (vy + (l*w)/2)/vx))/(2*Iz), cos(p2 + d*p1)*(d - (vy + (l*w)/2)/vx)*(1/m + (lf*(l/2 - lf))/Iz) - (pr*(vy - (l*w)/2)*(1/m - ((l - lf)*(l/2 - lf))/Iz))/vx, -(ca*(vy - (l*w)/2)*(1/m - ((l - lf)*(l/2 - lf))/Iz))/vx, -ca*d*sin(p2 + d*p1)*(d - (vy + (l*w)/2)/vx)*(1/m + (lf*(l/2 - lf))/Iz), -ca*sin(p2 + d*p1)*(d - (vy + (l*w)/2)/vx)*(1/m + (lf*(l/2 - lf))/Iz);...
                                                                                 0,                               -(ca*lf*cos(p2 + d*p1)*(d - (vy + (l*w)/2)/vx) + (ca*pr*(vy - (l*w)/2)*(l - lf))/vx)/Iz^2,                                     (ca*cos(p2 + d*p1)*(d - (vy + (l*w)/2)/vx) - (ca*pr*(vy - (l*w)/2))/vx)/Iz,                                                                                                                                                               (ca*(2*pr*vy - lf*w*cos(p2 + d*p1) - 2*l*pr*w + lf*pr*w))/(2*Iz*vx),                                            (lf*cos(p2 + d*p1)*(d - (vy + (l*w)/2)/vx) + (pr*(vy - (l*w)/2)*(l - lf))/vx)/Iz,                     (ca*(vy - (l*w)/2)*(l - lf))/(Iz*vx),                    -(ca*d*lf*sin(p2 + d*p1)*(d - (vy + (l*w)/2)/vx))/Iz,                    -(ca*lf*sin(p2 + d*p1)*(d - (vy + (l*w)/2)/vx))/Iz];
                                                                             
%add scaling to jacobian for parameters 
m=[1,.1,.1,.1,10,1,1,1];
m=repmat(m,5,1);
dfdp=m.*dfdp; 
end