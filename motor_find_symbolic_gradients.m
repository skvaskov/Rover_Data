clear
clc
syms Volt v ic J b ke kt R cr L r p2 real
x = [v ic]' ;
p = [J b ke kt R L r cr p2]';

f = [-b/J*v+r*kt/J*ic-cr;...
     -ke/(L*r)*v-R/L*ic+p2/L*Volt];

f=simplify(f);
dfdx = simplify(jacobian(f,x)) ;
dfdp = simplify(jacobian(f,p)) ;