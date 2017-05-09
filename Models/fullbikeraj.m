function [dfdt,dfdz,dfdp] = fullbikeraj(z,u,p)
%linear tire forces, proportional cruise control for lonigtudinal velocty
%states
x=z(1);
y=z(2);
psi=z(3);
vx=z(4);
vy=z(5);
w=z(6);
%inputs
d=u(1);
u2=u(2);
%parameters p = [m,Iz,lf,l,ca,a,b,c,kp,ps]';
scale=[1,.1,.1,.1,10,...
      1,.1,1e-4,.1,.01];
m=p(1)*scale(1); %mass [kg]
Iz=p(2)*scale(2); %moment of inertia yaw [kg m^2]
lf=p(3)*scale(3); %length of center of mass from front [m]
l=p(4)*scale(4); %wheelbase [m]
ca=p(5)*scale(5); %cornering stiffness [N]
a=p(6)*scale(6); %cruise control constant [m s^-2]
b=p(7)*scale(7);%cruise control constant [s^-1]
c=p(8)*scale(8); %cruise control constant [m^-1]
kp=p(9)*scale(9); %gain for throttle input
ps=p(10)*scale(10); %scaling factor for throttle input

dfdt= [                                                                                               vx*cos(psi) - vy*sin(psi);...
                                                                                                     vy*cos(psi) + vx*sin(psi);...
                                                                                                                             w;...
                           w^2*(l/2 - lf) - a - b*vx + vy*w - c*vx^2 - kp*(vx - ps*u2) - (ca*sin(d)*(d - (vy + (l*w)/2)/vx))/m;...
 ca*cos(d)*(d - (vy + (l*w)/2)/vx)*(1/m - (lf*(l/2 - lf))/Iz) - vx*w - (ca*(vy - (l*w)/2)*(1/m + ((l - lf)*(l/2 - lf))/Iz))/vx;...
                                                   (ca*lf*cos(d)*(d - (vy + (l*w)/2)/vx) + (ca*(vy - (l*w)/2)*(l - lf))/vx)/Iz];
                                               

 dfdz=[ 0, 0, - vy*cos(psi) - vx*sin(psi),                                                                                                                   cos(psi),                                                                              -sin(psi),                                                                                                     0;...
 0, 0,   vx*cos(psi) - vy*sin(psi),                                                                                                                   sin(psi),                                                                               cos(psi),                                                                                                     0;...
 0, 0,                           0,                                                                                                                          0,                                                                                      0,                                                                                                     1;...
 0, 0,                           0,                                                                    - b - kp - 2*c*vx - (ca*sin(d)*(vy + (l*w)/2))/(m*vx^2),                                                                 w + (ca*sin(d))/(m*vx),                                                          vy + 2*w*(l/2 - lf) + (ca*l*sin(d))/(2*m*vx);...
 0, 0,                           0, (ca*(vy - (l*w)/2)*(1/m + ((l - lf)*(l/2 - lf))/Iz))/vx^2 - w + (ca*cos(d)*(vy + (l*w)/2)*(1/m - (lf*(l/2 - lf))/Iz))/vx^2, - (ca*(1/m + ((l - lf)*(l/2 - lf))/Iz))/vx - (ca*cos(d)*(1/m - (lf*(l/2 - lf))/Iz))/vx, (ca*l*(1/m + ((l - lf)*(l/2 - lf))/Iz))/(2*vx) - vx - (ca*l*cos(d)*(1/m - (lf*(l/2 - lf))/Iz))/(2*vx);...
 0, 0,                           0,                                               -((ca*(vy - (l*w)/2)*(l - lf))/vx^2 - (ca*lf*cos(d)*(vy + (l*w)/2))/vx^2)/Iz,                                                     -(ca*(lf - l + lf*cos(d)))/(Iz*vx),                                                                -(ca*l*(l - lf + lf*cos(d)))/(2*Iz*vx)];
                                            
  
dfdp= [                                                                      0,                                                                                                          0,                                                                                                 0,                                                                                                                                                                                                           0,                                                                                                                0,  0,   0,     0,          0,     0;...
                                                                     0,                                                                                                          0,                                                                                                 0,                                                                                                                                                                                                           0,                                                                                                                0,  0,   0,     0,          0,     0;...
                                                                      0,                                                                                                          0,                                                                                                 0,                                                                                                                                                                                                           0,                                                                                                                0,  0,   0,     0,          0,     0;...
                                (ca*sin(d)*(d - (vy + (l*w)/2)/vx))/m^2,                                                                                                          0,                                                                                              -w^2,                                                                                                                                                                              w^2/2 + (ca*sin(d)*w)/(2*m*vx),                                                                              -(sin(d)*(d - (vy + (l*w)/2)/vx))/m, -1, -vx, -vx^2, ps*u2 - vx, kp*u2;...
 (ca*(vy - (l*w)/2))/(m^2*vx) - (ca*cos(d)*(d - (vy + (l*w)/2)/vx))/m^2, (ca*lf*cos(d)*(d - (vy + (l*w)/2)/vx)*(l/2 - lf))/Iz^2 + (ca*(vy - (l*w)/2)*(l - lf)*(l/2 - lf))/(Iz^2*vx), (ca*(2*vy - l*w)*(3*l - 4*lf))/(4*Iz*vx) + (ca*cos(d)*(l - 4*lf)*(2*vy - 2*d*vx + l*w))/(4*Iz*vx), (ca*w*(1/m + ((l - lf)*(l/2 - lf))/Iz))/(2*vx) - (ca*(vy - (l*w)/2)*((l - lf)/(2*Iz) + (l/2 - lf)/Iz))/vx - (ca*w*cos(d)*(1/m - (lf*(l/2 - lf))/Iz))/(2*vx) - (ca*lf*cos(d)*(d - (vy + (l*w)/2)/vx))/(2*Iz), cos(d)*(d - (vy + (l*w)/2)/vx)*(1/m - (lf*(l/2 - lf))/Iz) - ((vy - (l*w)/2)*(1/m + ((l - lf)*(l/2 - lf))/Iz))/vx,  0,   0,     0,          0,     0;...
                                                                      0,                             -(ca*lf*cos(d)*(d - (vy + (l*w)/2)/vx) + (ca*(vy - (l*w)/2)*(l - lf))/vx)/Iz^2,                                   (ca*cos(d)*(d - (vy + (l*w)/2)/vx) - (ca*(vy - (l*w)/2))/vx)/Iz,                                                                                                                                                          (ca*(2*vy - 2*l*w + lf*w - lf*w*cos(d)))/(2*Iz*vx),                                            (lf*cos(d)*(d - (vy + (l*w)/2)/vx) + ((vy - (l*w)/2)*(l - lf))/vx)/Iz,  0,   0,     0,          0,     0];
                                    

scale=repmat(scale,6,1);
dfdp=scale.*dfdp;

end

