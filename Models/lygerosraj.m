function [dzdt,dfdz,dfdp]=lygerosraj(z,u,p)
%dyanmic bike model from lygeros paper with small angle approximations for
%tire force angles

%states
x=z(1);
y=z(2);
psi=z(3);%heading
vy=z(4);%lateral velocity
w=z(5);%yaw rate
%p = [m,Iz,lf,l,Ca]';
scale=[1,.1,.1,.1,10];
m=p(1)*scale(1); %mass
Iz=p(2)*scale(2); %moment of inertia
lf=p(3)*scale(3); %lf (length of center of gravity from front wheel)
l=p(4)*scale(4);  %wheelbase length
Ca=p(5)*scale(5); %cornering stifness



%input
d=u(1); %steering input
vx=u(2); %longitudal velocity

dzdt=[vx*cos(psi) - vy*sin(psi);...
                                                                                                               vy*cos(psi) + vx*sin(psi);...
                                                                                                                                       w;...
 Ca*cos(d)*(1/m - (lf*(l/2 - lf))/Iz)*(d - atan((vy + (l*w)/2)/vx)) - Ca*atan((vy - (l*w)/2)/vx)*(1/m + ((l - lf)*(l/2 - lf))/Iz) - vx*w;...
                                                   (Ca*atan((vy - (l*w)/2)/vx)*(l - lf) + Ca*lf*cos(d)*(d - atan((vy + (l*w)/2)/vx)))/Iz];
 
dfdz=[ 0, 0, - vy*cos(psi) - vx*sin(psi),                                                                                                                                          -sin(psi),                                                                                                                                                             0;...
 0, 0,   vx*cos(psi) - vy*sin(psi),                                                                                                                                           cos(psi),                                                                                                                                                             0;...
 0, 0,                           0,                                                                                                                                                  0,                                                                                                                                                             1;...
 0, 0,                           0, - (Ca*(1/m + ((l - lf)*(l/2 - lf))/Iz))/(vx*((vy - (l*w)/2)^2/vx^2 + 1)) - (Ca*cos(d)*(1/m - (lf*(l/2 - lf))/Iz))/(vx*((vy + (l*w)/2)^2/vx^2 + 1)), (Ca*l*(1/m + ((l - lf)*(l/2 - lf))/Iz))/(2*vx*((vy - (l*w)/2)^2/vx^2 + 1)) - vx - (Ca*l*cos(d)*(1/m - (lf*(l/2 - lf))/Iz))/(2*vx*((vy + (l*w)/2)^2/vx^2 + 1));...
 0, 0,                           0,                                              ((Ca*(l - lf))/(vx*((vy - (l*w)/2)^2/vx^2 + 1)) - (Ca*lf*cos(d))/(vx*((vy + (l*w)/2)^2/vx^2 + 1)))/Iz,                                                -((Ca*l*(l - lf))/(2*vx*((vy - (l*w)/2)^2/vx^2 + 1)) + (Ca*l*lf*cos(d))/(2*vx*((vy + (l*w)/2)^2/vx^2 + 1)))/Iz];
 
dfdp= [                                                                                0,                                                                                                                    0,                                                                                                              0,                                                                                                                                                                                                                                                                             0,                                                                                                                          0;...
                                                                                0,                                                                                                                    0,                                                                                                              0,                                                                                                                                                                                                                                                                             0,                                                                                                                          0;...
                                                                                0,                                                                                                                    0,                                                                                                              0,                                                                                                                                                                                                                                                                             0,                                                                                                                          0;...
 (Ca*atan((vy - (l*w)/2)/vx))/m^2 - (Ca*cos(d)*(d - atan((vy + (l*w)/2)/vx)))/m^2, (Ca*atan((vy - (l*w)/2)/vx)*(l - lf)*(l/2 - lf))/Iz^2 + (Ca*lf*cos(d)*(d - atan((vy + (l*w)/2)/vx))*(l/2 - lf))/Iz^2, (Ca*atan((vy - (l*w)/2)/vx)*(3*l - 4*lf))/(2*Iz) - (Ca*cos(d)*(l - 4*lf)*(d - atan((vy + (l*w)/2)/vx)))/(2*Iz), (Ca*w*(1/m + ((l - lf)*(l/2 - lf))/Iz))/(2*vx*((vy - (l*w)/2)^2/vx^2 + 1)) - Ca*atan((vy - (l*w)/2)/vx)*((l - lf)/(2*Iz) + (l/2 - lf)/Iz) - (Ca*lf*cos(d)*(d - atan((vy + (l*w)/2)/vx)))/(2*Iz) - (Ca*w*cos(d)*(1/m - (lf*(l/2 - lf))/Iz))/(2*vx*((vy + (l*w)/2)^2/vx^2 + 1)), cos(d)*(1/m - (lf*(l/2 - lf))/Iz)*(d - atan((vy + (l*w)/2)/vx)) - atan((vy - (l*w)/2)/vx)*(1/m + ((l - lf)*(l/2 - lf))/Iz);...
                                                                                0,                             -(Ca*atan((vy - (l*w)/2)/vx)*(l - lf) + Ca*lf*cos(d)*(d - atan((vy + (l*w)/2)/vx)))/Iz^2,                                     -(Ca*atan((vy - (l*w)/2)/vx) - Ca*cos(d)*(d - atan((vy + (l*w)/2)/vx)))/Iz,                                                                                                                                   -((Ca*w*(l - lf))/(2*vx*((vy - (l*w)/2)^2/vx^2 + 1)) - Ca*atan((vy - (l*w)/2)/vx) + (Ca*lf*w*cos(d))/(2*vx*((vy + (l*w)/2)^2/vx^2 + 1)))/Iz,                                            (atan((vy - (l*w)/2)/vx)*(l - lf) + lf*cos(d)*(d - atan((vy + (l*w)/2)/vx)))/Iz];
 %add scaling to jacobian for parameters 

scale=repmat(scale,5,1);
dfdp=scale.*dfdp; 
end