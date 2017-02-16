function [dzdt,dfdz,dfdp]=lygerosreduced(z,u,p)
%states

VY=z(1);%lateral velocity
r=z(2);%yaw rate

m=p(1); %mass
Iz=p(2)/100; %moment of inertia P(2) is (kg*m^2)*100
lf=p(3)/10; %lf (length of center of gravity from front wheel) p(3) is m*10
l=p(4)/10;  %wheelbase length p(4) is m*10
caf=p(5)*10; %cornering stifness front
pca=p(6); %scaling for front and rear tire stiffness (car=caf*pca)


%input
u1=u(1); %steering input
u2=u(2); %longitudal velocity

dzdt=[    -(m*r*u2 + caf*pca*atan((VY - r*(l - lf))/u2) - caf*atan(u1 - (VY + lf*r)/u2)*cos(u1))/m;...
 (caf*lf*atan(u1 - (VY + lf*r)/u2)*cos(u1) + caf*pca*atan((VY - r*(l - lf))/u2)*(l - lf))/Iz];
  
dfdz=[             -((caf*pca)/(u2*((VY - r*(l - lf))^2/u2^2 + 1)) + (caf*cos(u1))/(u2*((u1 - (VY + lf*r)/u2)^2 + 1)))/m, -(m*u2 - (caf*pca*(l - lf))/(u2*((VY - r*(l - lf))^2/u2^2 + 1)) + (caf*lf*cos(u1))/(u2*((u1 - (VY + lf*r)/u2)^2 + 1)))/m;...
 ((caf*pca*(l - lf))/(u2*((VY - r*(l - lf))^2/u2^2 + 1)) - (caf*lf*cos(u1))/(u2*((u1 - (VY + lf*r)/u2)^2 + 1)))/Iz,   -((caf*lf^2*cos(u1))/(u2*((u1 - (VY + lf*r)/u2)^2 + 1)) + (caf*pca*(l - lf)^2)/(u2*((VY - r*(l - lf))^2/u2^2 + 1)))/Iz];
 

dfdp= [ (caf*(pca*atan((VY - r*(l - lf))/u2) - atan(u1 - (VY + lf*r)/u2)*cos(u1)))/m^2,                                                                                              0,                                                                                           -((caf*pca*r)/(u2*((VY - r*(l - lf))^2/u2^2 + 1)) + (caf*r*cos(u1))/(u2*((u1 - (VY + lf*r)/u2)^2 + 1)))/m,                                                  (caf*pca*r)/(m*u2*((VY - r*(l - lf))^2/u2^2 + 1)), 0,          -(caf*atan((VY - r*(l - lf))/u2))/m;...
                                                                          0, -(caf*lf*atan(u1 - (VY + lf*r)/u2)*cos(u1) + caf*pca*atan((VY - r*(l - lf))/u2)*(l - lf))/Iz^2, -(caf*pca*atan((VY - r*(l - lf))/u2) - caf*atan(u1 - (VY + lf*r)/u2)*cos(u1) + (caf*lf*r*cos(u1))/(u2*((u1 - (VY + lf*r)/u2)^2 + 1)) - (caf*pca*r*(l - lf))/(u2*((VY - r*(l - lf))^2/u2^2 + 1)))/Iz, (caf*pca*atan((VY - r*(l - lf))/u2) - (caf*pca*r*(l - lf))/(u2*((VY - r*(l - lf))^2/u2^2 + 1)))/Iz, 0, (caf*atan((VY - r*(l - lf))/u2)*(l - lf))/Iz];
 
end