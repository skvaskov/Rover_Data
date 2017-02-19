function [dzdt,dfdz,dudp] = simplepos(z,u,p)
%u1 heading u2 velocity x u3 B
dzdt=[u(2)*cos(u(1))-u(2)*tan(u(3))*sin(u(1));u(2)*sin(u(1))+u(2)*tan(u(3))*cos(u(1))];

dfdz=[0,0;0,0];

dudp=[];

% dzdt=[u(2)*cos(p(1)*u(1)+p(2));u(2)*sin(p(3)*u(1)+p(4))];
% 
% dfdz=[0,0;0,0];
% 
% dudp=[-u(1)*u(2)*sin(p(1)*u(1)+p(2)),-u(2)*sin(p(1)*u(1)+p(2)),0,0;...
%     0,0,u(1)*u(2)*cos(p(3)*u(1)+P(4)),u(2)*cos(p(3)*u(1)+P(4))];


end

