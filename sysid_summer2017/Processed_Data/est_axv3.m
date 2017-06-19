function [ax] = est_axv3(P,vx,const)
%input throttle input (P), longitudinal velocity (vx), and nx1 constants for the model
%output. acceleration (ax) in nx1 column vecort 
A=[ones(size(P)) P vx P.*vx vx.^2 P.^2 vx.*P.^2 P.^3];
ax=A*const;

end

