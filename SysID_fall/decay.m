function [dzdt,dfdz,dfdp] = decay(z,u,p)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
cr=p(1);
cd=p(2);
VX=z(1);

dzdt=[-cr-cd*VX^2];

dfdz=[-2*cd*VX];

dfdp=[-1,-VX^2];

end

