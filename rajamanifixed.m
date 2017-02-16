function [dzdt, dfdz, dfdp] = rajamanifixed(z,u,p)
  B=z(1);%sideslip
   r=z(2);%yaw rate 
m=2.470+.289; %mass
Iz=p(1); %moment of inertia
lf=p(2); %lf (length of center of gravity from front wheel)
l=.29; %lr (length of center of gravity from rear wheel)
ca=p(3); %cornering stiffness
p1=p(4); %scaling factor for steering input
u1=u(1); %steering input
u2=u(2); %longitudal velocity

dzdt=    [- r - (ca*(B - (r*(l - lf))/u2))/(m*u2) - (ca*(B - p1*u1 + (lf*r)/u2))/(m*u2);...
   (ca*(l - lf)*(B - (r*(l - lf))/u2))/Iz - (ca*lf*(B - p1*u1 + (lf*r)/u2))/Iz];

dfdz=[     -(2*ca)/(m*u2),   -(m*u2^2 - ca*l + 2*ca*lf)/(m*u2^2);...
 (ca*(l - 2*lf))/Iz, -(ca*(l^2 - 2*l*lf + 2*lf^2))/(Iz*u2)];

dfdp=[                                                                               0,                                 -(2*ca*r)/(m*u2^2),                          -(2*B*u2 - l*r + 2*lf*r - p1*u1*u2)/(m*u2^2), (ca*u1)/(m*u2);...
 (ca*lf*(B - p1*u1 + (lf*r)/u2))/Iz^2 - (ca*(l - lf)*(B - (r*(l - lf))/u2))/Iz^2, -(ca*(2*B*u2 - 2*l*r + 4*lf*r - p1*u1*u2))/(Iz*u2), ((l - lf)*(B - (r*(l - lf))/u2))/Iz - (lf*(B - p1*u1 + (lf*r)/u2))/Iz,  (ca*lf*u1)/Iz];
 
end