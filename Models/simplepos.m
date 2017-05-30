function [dzdt,dfdz,dudp] = simplepos(z,u,p)
psi=u(1);
vx=u(2);
vy=u(3);
dzdt=[vx*cos(psi)-vy*sin(psi);vx*sin(psi)+vy*cos(psi)];

dfdz=[0,0;0,0];

dudp=[];

% dzdt=[u(2)*cos(p(1)*u(1)+p(2));u(2)*sin(p(3)*u(1)+p(4))];
% 
% dfdz=[0,0;0,0];
% 
% dudp=[-u(1)*u(2)*sin(p(1)*u(1)+p(2)),-u(2)*sin(p(1)*u(1)+p(2)),0,0;...
%     0,0,u(1)*u(2)*cos(p(3)*u(1)+P(4)),u(2)*cos(p(3)*u(1)+P(4))];


end

