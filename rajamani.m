function [dzdt, dfdz, dfdp] = rajamanifull(z,u,p)
%states
X=z(1);
Y=z(2);
H=z(3);%heading
B=z(4);%sideslip angle
r=z(5);%yaw rate

m=p(1); %mass
Iz=p(2); %moment of inertia
lf=p(3); %lf (length of center of gravity from front wheel)
l=p(4);  %wheelbase length
ca=p(5); %cornering stifness
p1=p(6);%scaling factor for input

%input
u1=u(1); %steering input
u2=u(2); %longitudal velocity


dzdt=[u2*cos(H) - u2*tan(B)*sin(H);...
       u2*sin(H) + u2*cos(H)*tan(B);...
      r;...
 - r - (ca*(B - (r*(l - lf))/u2))/(m*u2) - (ca*(B - p1*u1 + (lf*r)/u2))/(m*u2);...
   (ca*(l - lf)*(B - (r*(l - lf))/u2))/Iz - (ca*lf*(B - p1*u1 + (lf*r)/u2))/Iz];

dfdz= [ 0, 0, -(u2*sin(B + H))/cos(B), (u2*sin(H))/(sin(B)^2 - 1),                                     0;...
 0, 0,  (u2*cos(B + H))/cos(B),       (u2*cos(H))/cos(B)^2,                                     0;...
 0, 0,                       0,                          0,                                     1;...
 0, 0,                       0,             -(2*ca)/(m*u2),   -(m*u2^2 - ca*l + 2*ca*lf)/(m*u2^2);...
 0, 0,                       0,         (ca*(l - 2*lf))/Iz, -(ca*(l^2 - 2*l*lf + 2*lf^2))/(Iz*u2)];
  
    

dfdp=[                                                  0,                                                                               0,                                                  0,                                    0,                                                                     0,              0;...
                                                  0,                                                                               0,                                                  0,                                    0,                                                                     0,              0;...
                                                  0,                                                                               0,                                                  0,                                    0,                                                                     0,              0;...
 (ca*(2*B*u2 - l*r + 2*lf*r - p1*u1*u2))/(m^2*u2^2),                                                                               0,                                 -(2*ca*r)/(m*u2^2),                      (ca*r)/(m*u2^2),                          -(2*B*u2 - l*r + 2*lf*r - p1*u1*u2)/(m*u2^2), (ca*u1)/(m*u2);...
                                                  0, (ca*lf*(B - p1*u1 + (lf*r)/u2))/Iz^2 - (ca*(l - lf)*(B - (r*(l - lf))/u2))/Iz^2, -(ca*(2*B*u2 - 2*l*r + 4*lf*r - p1*u1*u2))/(Iz*u2), (ca*(B*u2 - 2*l*r + 2*lf*r))/(Iz*u2), ((l - lf)*(B - (r*(l - lf))/u2))/Iz - (lf*(B - p1*u1 + (lf*r)/u2))/Iz,  (ca*lf*u1)/Iz];
 
end