function [dZdt] = bikeconstv(t,Z,steerfun,vxfun,P)
%bicycle model with constant velocity and steering angle
%Parameters
l=P(1);
d=P(2);

%input vector U = desired steering angle, desired velocity
    gammades=steerfun(t);
    vxdes=vxfun(t); 
%states
x=Z(1);
y=Z(2);
psi=Z(3);

%dynamics
dZdt=[(cos(psi)-d/l*sin(psi)*tan(gammades))*vxdes;...
    (sin(psi)+d/l*cos(psi)*tan(gammades))*vxdes;
    1/l*tan(gammades)*vxdes];
end

