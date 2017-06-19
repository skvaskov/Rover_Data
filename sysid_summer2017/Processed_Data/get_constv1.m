function [const] = get_constv1(P,vx,ax)
%input throttle input (P), longitudinal velocity (vx), and acceleration
%(ax) in nx1 column vecort
%output. nx1 constants for the model ax=const*[-1 P -P*vx -vx^2]
A=[-ones(size(P)) P -P.*vx -vx.^2];
const=A\ax;

end

