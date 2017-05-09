function [dzdt, dfdz,dfdp]=unicycle(z,u,p)
%states
x=z(1);
y=z(2);
psi=z(3);
%input
v=u(1); %velocity
w=u(2); %angular velocity

dzdt=[v*cos(psi);v*sin(psi);w];

dfdz=[0,0,-v*sin(psi);0,0,v*cos(psi);0,0,0];

dfdp=0;

end