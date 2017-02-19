clear
clc
%%
syms X Y B r H m Iz lf l ca p1 u1 u2 real
syms x_0 v_0 real
x = [B r]' ;
p = [Iz lf ca p1]' ;

%%
f = [-r+ca/m/u2*(p1*u1-B-lf*r/u2)+ca/m/u2*(-B+(l-lf)*r/u2);...
    lf*ca/Iz*(p1*u1-B-lf*r/(u2))-(l-lf)*ca/Iz*(-B+(l-lf)*r/(u2))];
 
%%
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