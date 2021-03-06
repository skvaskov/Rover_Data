clear
clc
%%
syms x y psi ca Bf a b c kp ps Cf Df k Ef Sh mu muo Sv prB prC prD vy vx  d Iz lf l lr u1 u2 v w p1 p2 s1 s2 s3 s4 s5 s6 s7 s8 s9 m real
syms x_0 v_0 real
x = [x y psi vx vy w]' ;
p = [m,Iz,lf,l,ca,a,b,c,kp,ps]';
%b = atan((l-lf)/l*tan(d));

 slipf=d-atan((vy+w*l/2)/vx);
 xf=slipf-Sh;
 slipr=atan((vy-w*(l/2))/vx);
 xr=slipr-Sh;
 
Fyf=ca*(d-(w*l/2+vy)/vx);
Fyr=ca*(w*l/2-vy)/vx;
 %%
 f=[vx*cos(psi)-vy*sin(psi);...
     vx*sin(psi)+vy*cos(psi);...
     w;...
     -a-b*vx-c*vx^2+kp*(ps*u2-vx)-1/m*Fyf*sin(d)+vy*w-(lf-l/2)*w^2;...
     (1/(m)+(l/2-lf)*(l-lf)/(Iz))*Fyr+(1/(m)-(l/2-lf)*lf/(Iz))*cos(d)*Fyf-vx*w;...
     1/(Iz)*(Fyf*lf*cos(d)-Fyr*(l-lf))];
%     
% % % 
% f=[(cos(psi)-1/2*sin(psi)*tan(p1*d+p2))*vx;...
%      (sin(psi)+1/2*cos(psi)*tan(p1*d+p2))*vx;...
%      psidot];



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