function [dzdt,dfdz,dfdp]=lygerostan(z,u,p)
%states
X=z(1);
Y=z(2);
psi=z(3);%heading
vy=z(4);%lateral velocity
w=z(5);%yaw rate

m=p(1); %mass
Iz=p(2)/10; %moment of inertia
lf=p(3)/10; %lf (length of center of gravity from front wheel)
l=p(4)/10;  %wheelbase length
ca=p(5); %cornering stifness
pr=p(6);%scaling factor for rear tire stifness
p1=p(7);
p2=p(8);

%input
d=u(1); %steering input
p1=p(7);%scaling factor
p2=p(8);%scaling factor
vx=u(2); %longitudal velocity
dzdt= [    vx*cos(psi) - vy*sin(psi) + (vx*tan(p2 + d*p1)*sin(psi)*(l/2 - lf))/l;...
                                     vy*cos(psi) + vx*sin(psi) - (vx*tan(p2 + d*p1)*cos(psi)*(l/2 - lf))/l;...
                                                                                                         w;...
      (ca*pr*atan2(vy - w*(l - lf), vx) - m*vx*w + ca*cos(p2 + d*p1)*(p2 - atan2(vy + lf*w, vx) + d*p1))/m;...
 -(ca*pr*atan2(vy - w*(l - lf), vx)*(l - lf) - ca*lf*cos(p2 + d*p1)*(p2 - atan2(vy + lf*w, vx) + d*p1))/Iz];
  
dfdz= [ 0, 0, (vx*tan(p2 + d*p1)*cos(psi)*(l/2 - lf))/l - vx*sin(psi) - vy*cos(psi),                                                                                                 -sin(psi),                                                                                                               0;...
 0, 0, vx*cos(psi) - vy*sin(psi) + (vx*tan(p2 + d*p1)*sin(psi)*(l/2 - lf))/l,                                                                                                  cos(psi),                                                                                                               0;...
 0, 0,                                                                     0,                                                                                                         0,                                                                                                               1;...
 0, 0,                                                                     0,              -((ca*vx*cos(p2 + d*p1))/((vy + lf*w)^2 + vx^2) - (ca*pr*vx)/((vy - w*(l - lf))^2 + vx^2))/m, -(m*vx + (ca*lf*vx*cos(p2 + d*p1))/((vy + lf*w)^2 + vx^2) + (ca*pr*vx*(l - lf))/((vy - w*(l - lf))^2 + vx^2))/m;...
 0, 0,                                                                     0, -((ca*lf*vx*cos(p2 + d*p1))/((vy + lf*w)^2 + vx^2) + (ca*pr*vx*(l - lf))/((vy - w*(l - lf))^2 + vx^2))/Iz,   -((ca*lf^2*vx*cos(p2 + d*p1))/((vy + lf*w)^2 + vx^2) - (ca*pr*vx*(l - lf)^2)/((vy - w*(l - lf))^2 + vx^2))/Iz];
 
dfdp= [                                                                                                                   0,                                                                                                          0,                                                                                                                                                                        -(vx*tan(p2 + d*p1)*sin(psi))/l,                                                         (lf*vx*tan(p2 + d*p1)*sin(psi))/l^2,                                                                                                   0,                                            0,                                     (d*vx*sin(psi)*(tan(p2 + d*p1)^2 + 1)*(l/2 - lf))/l,                                   (vx*sin(psi)*(tan(p2 + d*p1)^2 + 1)*(l/2 - lf))/l;...
                                                                                                                   0,                                                                                                          0,                                                                                                                                                                         (vx*tan(p2 + d*p1)*cos(psi))/l,                                                        -(lf*vx*tan(p2 + d*p1)*cos(psi))/l^2,                                                                                                   0,                                            0,                                    -(d*vx*cos(psi)*(tan(p2 + d*p1)^2 + 1)*(l/2 - lf))/l,                                  -(vx*cos(psi)*(tan(p2 + d*p1)^2 + 1)*(l/2 - lf))/l;...
                                                                                                                   0,                                                                                                          0,                                                                                                                                                                                                      0,                                                                                           0,                                                                                                   0,                                            0,                                                                                       0,                                                                                   0;...
 - (ca*pr*atan2(vy - w*(l - lf), vx) - m*vx*w + ca*cos(p2 + d*p1)*(p2 - atan2(vy + lf*w, vx) + d*p1))/m^2 - (vx*w)/m,                                                                                                          0,                                                                                                       -((ca*vx*w*cos(p2 + d*p1))/((vy + lf*w)^2 + vx^2) - (ca*pr*vx*w)/((vy - w*(l - lf))^2 + vx^2))/m,                                              -(ca*pr*vx*w)/(m*((vy - w*(l - lf))^2 + vx^2)),               (pr*atan2(vy - w*(l - lf), vx) + cos(p2 + d*p1)*(p2 - atan2(vy + lf*w, vx) + d*p1))/m,            (ca*atan2(vy - w*(l - lf), vx))/m,        (ca*d*cos(p2 + d*p1) - ca*d*sin(p2 + d*p1)*(p2 - atan2(vy + lf*w, vx) + d*p1))/m,        (ca*cos(p2 + d*p1) - ca*sin(p2 + d*p1)*(p2 - atan2(vy + lf*w, vx) + d*p1))/m;...
                                                                                                                   0, (ca*pr*atan2(vy - w*(l - lf), vx)*(l - lf) - ca*lf*cos(p2 + d*p1)*(p2 - atan2(vy + lf*w, vx) + d*p1))/Iz^2, (ca*pr*atan2(vy - w*(l - lf), vx) + ca*cos(p2 + d*p1)*(p2 - atan2(vy + lf*w, vx) + d*p1) - (ca*lf*vx*w*cos(p2 + d*p1))/((vy + lf*w)^2 + vx^2) - (ca*pr*vx*w*(l - lf))/((vy - w*(l - lf))^2 + vx^2))/Iz, -(ca*pr*atan2(vy - w*(l - lf), vx) - (ca*pr*vx*w*(l - lf))/((vy - w*(l - lf))^2 + vx^2))/Iz, -(pr*atan2(vy - w*(l - lf), vx)*(l - lf) - lf*cos(p2 + d*p1)*(p2 - atan2(vy + lf*w, vx) + d*p1))/Iz, -(ca*atan2(vy - w*(l - lf), vx)*(l - lf))/Iz, (ca*d*lf*cos(p2 + d*p1) - ca*d*lf*sin(p2 + d*p1)*(p2 - atan2(vy + lf*w, vx) + d*p1))/Iz, (ca*lf*cos(p2 + d*p1) - ca*lf*sin(p2 + d*p1)*(p2 - atan2(vy + lf*w, vx) + d*p1))/Iz];
 
%add scaling to jacobian for parameters 
m=[1,.1,.1,.1,1,1,1,1];
m=repmat(m,5,1);
dfdp=m.*dfdp;
end