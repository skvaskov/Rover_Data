function [ax] = est_axv1(P,vx,const)
%input throttle input (P), longitudinal velocity (vx), and nx1 constants for the model ax=const*[-1 P -P*vx -vx^2]
%output. acceleration (ax) in nx1 column vecort 
A=[-ones(size(P)) P -P.*vx -vx.^2];
ax=A*const;

end

