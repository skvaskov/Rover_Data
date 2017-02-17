function [dzdt, dfdz, dfdp] = rajamanifull(z,u,p)
%states
X=z(1);
Y=z(2);
psi=z(3);%heading
B=z(4);%sideslip angle
w=z(5);%yaw rate

m=p(1); %mass
Iz=p(2); %moment of inertia
lf=p(3); %lf (length of center of gravity from front wheel)
l=p(4);  %wheelbase length
ca=p(5); %cornering stifness
pr=p(6);%scaling factor for input

%input
d=u(1); %steering input
vx=u(2); %longitudal velocity


dzdt=[(vx*cos(B + psi))/cos(B);...
                                                     (vx*sin(B + psi))/cos(B);...
                                                                            w;...
 - w - (ca*(B - d + (lf*w)/vx))/(m*vx) - (ca*pr*(B - (w*(l - lf))/vx))/(m*vx);...
   (ca*pr*(l - lf)*(B - (w*(l - lf))/vx))/Iz - (ca*lf*(B - d + (lf*w)/vx))/Iz];

dfdz= [ 0, 0, -(vx*sin(B + psi))/cos(B), (vx*sin(psi))/(sin(B)^2 - 1),                                                0;...
 0, 0,  (vx*cos(B + psi))/cos(B),       (vx*cos(psi))/cos(B)^2,                                                0;...
 0, 0,                         0,                            0,                                                1;...
 0, 0,                         0,        -(ca*(pr + 1))/(m*vx), (ca*pr*(l - lf))/(m*vx^2) - (ca*lf)/(m*vx^2) - 1;...
 0, 0,                         0, -(ca*(lf - l*pr + lf*pr))/Iz, - (ca*lf^2)/(Iz*vx) - (ca*pr*(l - lf)^2)/(Iz*vx)];
 
dfdp=[                                                                          0,                                                                              0,                                                                     0,                                       0,                                                                    0,                                      0;...
                                                                         0,                                                                              0,                                                                     0,                                       0,                                                                    0,                                      0;...
                                                                          0,                                                                              0,                                                                     0,                                       0,                                                                    0,                                      0;...
 (ca*(B - d + (lf*w)/vx))/(m^2*vx) + (ca*pr*(B - (w*(l - lf))/vx))/(m^2*vx),                                                                              0,                                             -(ca*w*(pr + 1))/(m*vx^2),                      (ca*pr*w)/(m*vx^2),     - (B - d + (lf*w)/vx)/(m*vx) - (pr*(B - (w*(l - lf))/vx))/(m*vx),     -(ca*(B - (w*(l - lf))/vx))/(m*vx);...
                                                                          0, (ca*lf*(B - d + (lf*w)/vx))/Iz^2 - (ca*pr*(l - lf)*(B - (w*(l - lf))/vx))/Iz^2, -(ca*(B*vx - d*vx + 2*lf*w + B*pr*vx - 2*l*pr*w + 2*lf*pr*w))/(Iz*vx), (ca*pr*(B*vx - 2*l*w + 2*lf*w))/(Iz*vx), (pr*(l - lf)*(B - (w*(l - lf))/vx))/Iz - (lf*(B - d + (lf*w)/vx))/Iz, (ca*(l - lf)*(B - (w*(l - lf))/vx))/Iz];
 

end