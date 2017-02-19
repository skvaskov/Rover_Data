function [dzdt,dfdz,dfdp]=lygerosfull(z,u,p)
%states
X=z(1);
Y=z(2);
H=z(3);%heading
VY=z(4);%lateral velocity
r=z(5);%yaw rate

p1=p(1); %(cm1-cr)/m
p2=p(2); %-cm2*d/m
p3=p(3); %-cd/m
p4=p(4);  %lf
p5=p(5); 
car=caf;
p1=p(6);%scaling factor for input

%input
u1=u(1); %steering input
u2=u(2); %longitudal velocity

dzdt=[                                                                 u2*cos(H) - VY*sin(H);...
                                                                 u2*sin(H) - VY*cos(H);...
                                                                                     r;...
    -(caf*cos(p1*u1)*((VY + lf*r)/u2 - p1*u1) + m*r*u2 + (car*(VY - r*(l - lf)))/u2)/m;...
 ((car*(VY - r*(l - lf))*(l - lf))/u2 - caf*lf*cos(p1*u1)*((VY + lf*r)/u2 - p1*u1))/Iz];
  
dfdz=[ 0, 0, - VY*cos(H) - u2*sin(H),                                         -sin(H),                                                      0;...
 0, 0,   VY*sin(H) + u2*cos(H),                                         -cos(H),                                                      0;...
 0, 0,                       0,                                               0,                                                      1;...
 0, 0,                       0,                  -(car + caf*cos(p1*u1))/(m*u2), -(m*u2 - (car*(l - lf))/u2 + (caf*lf*cos(p1*u1))/u2)/m;...
 0, 0,                       0, ((car*(l - lf))/u2 - (caf*lf*cos(p1*u1))/u2)/Iz,   -((car*(l - lf)^2)/u2 + (caf*lf^2*cos(p1*u1))/u2)/Iz];
 

dfdp=[                                                                                              0,                                                                                        0,                                                                                                                0,                                   0,                                            0+                                    0,                                                                         0;...
                                                                                              0,                                                                                        0,                                                                                                                0,                                   0,                                            0+                                   0,                                                                         0;...
                                                                                              0,                                                                                        0,                                                                                                                0,                                   0,                                            0+                                    0,                                                                         0;...
 (caf*cos(p1*u1)*((VY + lf*r)/u2 - p1*u1) + m*r*u2 + (car*(VY - r*(l - lf)))/u2)/m^2 - (r*u2)/m,                                                                                        0,                                                                               -(r*(car + caf*cos(p1*u1)))/(m*u2),                      (car*r)/(m*u2),     -(cos(p1*u1)*((VY + lf*r)/u2 - p1*u1))/m+            -(VY - r*(l - lf))/(m*u2),        (caf*u1*cos(p1*u1) + caf*u1*sin(p1*u1)*((VY + lf*r)/u2 - p1*u1))/m;...
                                                                                              0, -((car*(VY - r*(l - lf))*(l - lf))/u2 - caf*lf*cos(p1*u1)*((VY + lf*r)/u2 - p1*u1))/Iz^2, -(VY*car - 2*car*l*r + 2*car*lf*r + VY*caf*cos(p1*u1) + 2*caf*lf*r*cos(p1*u1) - caf*p1*u1*u2*cos(p1*u1))/(Iz*u2), (car*(VY - 2*l*r + 2*lf*r))/(Iz*u2), -(lf*cos(p1*u1)*((VY + lf*r)/u2 - p1*u1))/Iz+ ((VY - r*(l - lf))*(l - lf))/(Iz*u2), (caf*lf*u1*cos(p1*u1) + caf*lf*u1*sin(p1*u1)*((VY + lf*r)/u2 - p1*u1))/Iz];
 
end