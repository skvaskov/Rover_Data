clear
clc
%%
syms x_1 x_2 x_3 x_4 p_1 p_2 p_3 p_4 u1 u2 real
syms x_0 y_0 h_0 v_0 real
x = [x_1 x_2 x_3 x_4]' ;
p = [p_1 p_2 p_3 p_4]' ;

b = atan((p_3/(p_3+p_4))*tan(u1)) ;


%%
f = [p_1*x_4*cos(x_3+b) ;
     p_2*x_4*sin(x_3+b) ;
     (x_4*cos(b)*tan(u1))/(p_3+p_4) ;
     u2] ;
 
%%
dfdx = jacobian(f,x) ;

%%
dfdp = simplify(jacobian(f,p)) ;

%%
syms B real
fB = [p_1*x_4*cos(x_3+B) ;
      p_2*x_4*sin(x_3+B) ;
      (x_4*cos(B)*tan(u1))/(p_3+p_4) ;
      u2] ;
  
%%
fTaylor = simplify(taylor(fB,x,[x_0,y_0,h_0,v_0]','Order',5)) ;

dfdxTaylor = simplify(jacobian(f,x)) ;
dfdpTaylor = simplify(jacobian(f,p)) ;