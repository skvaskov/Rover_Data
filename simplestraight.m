function [dzdt, dfdz, dfdp] = simplestraight(z,u,p)
x=z(1); %distance
v=z(2); %velocity

u1=u(1); %throttle

dzdt=[v;p(1)*u1+p(2)*v+p(3)*u1*v+p(4)];
dfdz=[0, 1;0, p(2)+p(3)*u1];
dfdp=[0, 0, 0,0;u1, v, u1*v,1];
end

