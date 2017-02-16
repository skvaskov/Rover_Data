clear
clc
%%
syms X Y H VY VX r m cm1 cm2 cd cr Iz lf l car caf pca p2 u1 u2 real
syms x_0 v_0 real
x = [VY r]' ;
p = [m Iz lf l car pca]';
b = atan((l-lf)/l*tan(u1)) ;
car=pca*caf;
Fyr=car*atan((r*(l-lf)-VY)/u2);
Fyf=caf*atan((u1-(r*lf+VY)/u2));
Fxr=(cm1-cm2*VX)*p2*u2-cr-cd*VX^2;

%%
% f = [u2*cos(H)-VY*sin(H);...
%     u2*sin(H)-VY*cos(H);...
%     r;...
%     1/m*(Fyr+Fyf*cos(p1*u1)-m*u2*r);...
%     1/Iz*(Fyf*lf*cos(p1*u1)-Fyr*(l-lf))];

f = [1/m*(Fyr+Fyf*cos(u1)-m*u2*r);...
    1/Iz*(Fyf*lf*cos(u1)-Fyr*(l-lf))];
% 
%  f = [VX/cos(b)*cos(H+b);...
%      VX/cos(b)*sin(H+b);...
%       VX/l*tan(p1*u1);...
%      1/m*(Fxr-Fyf*sin(p1*u1)+m*VX*tan(b)*VX/l*tan(p1*u1))];

% f=[VX;1/m*Fxr];
 
%%
f=simplify(f);
dfdx = simplify(jacobian(f,x)) ;
%%
dfdp = simplify(jacobian(f,p)) ;
%%
syms B real
fB = [p_1*x_4*cos(x_3+B) ;
      p_2*x_4*sin(x_3+B) ;
      (x_4*cos(B)*tan(u1))/(p_3+p_4) ;
      u2] ;
  
%%
fTaylor = simplify(taylor(fB,x,[x_0,0,0,v_0]','Order',5)) ;
dfdxTaylor = simplify(jacobian(f,x)) ;
dfdpTaylor = simplify(jacobian(f,p)) ;