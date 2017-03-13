clear
clc
%%
syms x y psi B Bf Cf Df prB prC prD vy vx r d m cm1 cm2 cd cr Iz lf l lr car caf ca pr u1 u2 v w p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 real
syms x_0 v_0 real
x = [x y psi vy w]' ;
p = [m,Iz,lf,l,Bf,Cf,Df,prB,prC,prD,p1,p2]';
%b = atan((l-lf)/l*tan(d));

 slipf=(vy+w*l/2)/vx;
 slipr=(-vy+w*(l/2))/vx;
 Fyr=prD*Df*sin(prC*Cf*atan(prB*Bf*slipr));
Fyf=Df*sin(Cf*atan(Bf*(d-slipf)));
%psidot=1/l*tan(p1*d+p2)*vx;
% Fxr=(cm1-cm2*VX)*p2*u2-cr-cd*VX^2;

 %%
 f=[vx*cos(psi)-vy*sin(psi);...
     vx*sin(psi)+vy*cos(psi);...
     w;...
     (1/m-(l/2-lf)*(l-lf)/Iz)*Fyr+(1/m+(l/2-lf)*lf/Iz)*cos(p1*d+p2)*Fyf-vx*w;...
     1/Iz*(Fyf*lf*cos(p1*d+p2)-Fyr*(l-lf))];
    
% % 
% f=[(cos(psi)-lr/l*sin(psi)*tan(p1*d+p2))*vx-psidot*(l/2-lr)*sin(psi);...
%     (sin(psi)+lr/l*cos(psi)*tan(p1*d+p2))*vx+psidot*(l/2-lr)*cos(psi);...
%     psidot];



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