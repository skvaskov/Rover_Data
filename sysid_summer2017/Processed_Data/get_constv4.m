function [const] = get_constv4(P,vx,ax)
%input throttle input (P), longitudinal velocity (vx), and acceleration
%(ax) in nx1 column vecort
%output. nx1 constants for the model 
A=[ones(size(P)) P vx P.*vx vx.^2 P.^2 vx.*P.^2 P.*vx.^2 vx.^3 P.^3 vx.^2.*P.^2 vx.*P.^3 P.*vx.^3 P.^4 vx.^4];
const=A\ax;

end

