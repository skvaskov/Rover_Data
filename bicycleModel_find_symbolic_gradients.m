clear
clc
%%
syms x y psi B vy vx r d m cm1 cm2 cd cr Iz lf l car caf ca pr p2 u1 u2 v w real
syms x_0 v_0 real
x = [x y psi vy w]' ;
p = [m,Iz,lf,l,ca,pr]';
%b = atan((l-lf)/l*tan(d));

 Fyr=pr*ca*(w*(l-lf)-vy)/vx;
 Fyf=ca*(d-(w*lf+vy)/vx);
% Fxr=(cm1-cm2*VX)*p2*u2-cr-cd*VX^2;

%%
f=[vx*cos(psi)-vy*sin(psi);
    vx*sin(psi)+vy*cos(psi);...
    w;...
    (1/m)*(Fyr+Fyf*cos(d)-m*vx*w);...
    (1/Iz)*(Fyf*lf*cos(d)-Fyr*(l-lf))];
% 

%%
%f=simplify(f);
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