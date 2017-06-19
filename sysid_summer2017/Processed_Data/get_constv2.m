function [const] = get_constv2(P,vx,ax)
%input throttle input (P), longitudinal velocity (vx), and acceleration
%(ax) in nx1 column vecort
%output. nx1 constants for the model 
A=[ones(size(P)) P vx P.*vx vx.^2 P.^2 vx.*P.^2];
const=A\ax;

end

