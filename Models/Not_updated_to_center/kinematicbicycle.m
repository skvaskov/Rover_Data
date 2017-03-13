function [dzdt,dfdz,dfdp] = kinematicbicycle(z,u,p)
%P=[lf l];
lf=p(1);
l=p(2);

x=z(1);
y=z(2);
psi=z(3);

d=u(1);
v=u(2);

dzdt=[               v*cos(psi + atan((tan(d)*(l - lf))/l));...
               v*sin(psi + atan((tan(d)*(l - lf))/l));...
 (v*tan(d))/(l*((tan(d)^2*(l - lf)^2)/l^2 + 1)^(1/2))];

dfdz=[ 0, 0, -v*sin(psi + atan((tan(d)*(l - lf))/l));...
 0, 0,  v*cos(psi + atan((tan(d)*(l - lf))/l));...
 0, 0,                                       0];

dfdp=[  (v*sin(psi + atan((tan(d)*(l - lf))/l))*tan(d))/(l*((tan(d)^2*(l - lf)^2)/l^2 + 1)), -(v*sin(psi + atan((tan(d)*(l - lf))/l))*(tan(d)/l - (tan(d)*(l - lf))/l^2))/((tan(d)^2*(l - lf)^2)/l^2 + 1);...
 -(v*cos(psi + atan((tan(d)*(l - lf))/l))*tan(d))/(l*((tan(d)^2*(l - lf)^2)/l^2 + 1)),  (v*cos(psi + atan((tan(d)*(l - lf))/l))*(tan(d)/l - (tan(d)*(l - lf))/l^2))/((tan(d)^2*(l - lf)^2)/l^2 + 1);...
              (v*tan(d)^3*(2*l - 2*lf))/(2*l^3*((tan(d)^2*(l - lf)^2)/l^2 + 1)^(3/2)),                       -(v*tan(d)*(l + l*tan(d)^2 - lf*tan(d)^2))/(l^3*((tan(d)^2*(l - lf)^2)/l^2 + 1)^(3/2))];
 
end

