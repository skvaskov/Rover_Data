function [dZdt] = bike_paratire_wx_feedback(t,Z,U,P)
%bicycle model with parabolic pressure distrubution tire forces. Combined tire model, Derive from Pacejka and Sharp,(Raj
%book page 384)
%states
x=Z(1);
y=Z(2);
psi=Z(3);
vx=Z(4);
wx=Z(5);
vy=Z(6);
w=Z(7);
xd=Z(8);
yd=Z(9);
psid=Z(10);

%input vector
gamma0=U(1);
gamma0dot=U(2);
vx0=U(3);

%parameter vector
%P=[m,Jg,l,d,avx, bvx, cvx,Rw, a,b,k,mu,mu0,ky,kpsi,kx,kvx,kwx]
g=9.80655;
m=P(1);
Jg=P(2);
l=P(3);
d=P(4);
avx=P(5);
bvx=P(6);
cvx=P(7);
Re=P(8);
a=P(9);
b=P(10);
k=P(11);
mu=P(12);
mu0=P(13);
ky=P(14);
kpsi=P(15);
kx=P(16);
kvx=P(17);
kwx=P(18);

%error
ex=cos(psid)*(xd-x)+sin(psid)*(yd-y);
ey=-sin(psid)*(xd-x)+cos(psid)*(yd-y);
eydot=(d-cos(psid)*(xd-x)-sin(psid)*(yd-y))*vx0/l*tan(gamma0)+...
    vx*sin(psid-psi)+vy*cos(psid-psi);


eh=psid-psi;
ehdot=vx0/l*tan(gamma0)-w;
evx=vx0-vx;

%input
gamma=ky*ey+kpsi*eh+gamma0;
gammadot=ky*eydot+kpsi*ehdot+gamma0dot;
wxdes=kx*ex+kvx*evx+vx0/Re;

%slip angles
if vx==0
    tar=0;
    taf=0;
else
tar=-(vy-(d-a)*w)/vx;
taf=-(-vx*sin(gamma)+(vy+(l-d)*w)*cos(gamma)+a*(w+gammadot))/(vx*cos(gamma)+(vy+(l-d)*w)*sin(gamma));

end
%slip ratios
if wxdes>=wx
    sigmax=(Re*wx-vx)/Re*wx;
else
    sigmax=(Re*wx-vx)/vx;
end
sigmayr=vx/(Re*wx)*tar;
sigmar=sqrt(sigmax^2+sigmayr^2);
sigmayf=taf;
sigmaf=sqrt(sigmayf^2);

%normal Forces (BIG ASSUMPTION that they are constant)
Fzf=m*g*d/l;
Fzr=m*g*(l-d)/l;
%limits for sliding
thetaf=4*a^2*b*k/(3*mu0*Fzf);
thetar=4*a^2*b*k/(3*mu0*Fzr);

%Lateral force and moment for front
if 0<sigmaf<=1/thetaf
    Fyf=sigmayf/sigmaf*mu*Fzf*(3*thetaf*sigmaf-1/3*(3*thetaf*sigmaf)^2+1/27*(3*thetaf*sigmaf)^3);
    Mf=sigmayf/sigmaf*mu*Fzf*a*(thetaf*sigmaf-3*thetaf^2*sigmaf^2+3*thetaf^3*sigmaf^3-thetaf^4*sigmaf^4);
elseif sigmaf==0
    Fyf=0;
    Mf=0;
else
    Fyf=mu*Fzf;
    Mf=0;
end

%Rear forces
if 0<sigmar&&sigmar<=1/thetar
Fyr=sigmayr/sigmar*mu*Fzr*(3*thetar*sigmar-1/3*(3*thetar*sigmar)^2+1/27*(3*thetar*sigmar)^3);
Fx=sigmax/sigmar*mu*Fzr*(3*thetar*sigmar-1/3*(3*thetar*sigmar)^2+1/27*(3*thetar*sigmar)^3);
Mr=mu*Fzr*a*sigmayr/sigmar*(thetar*sigmar-3*thetar^2*sigmar^2+3*thetar^3*sigmar^3-thetar^4*sigmar^4);
elseif sigmar==0
    Fx=0;
    Fyr=0;
    Mr=0;
else
    Fyr=sigmayr/sigmar*mu*Fzr;
    Fx=sigmax/sigmar*mu*Fzr;
    Mr=0;
end

R=-avx-bvx*vx-cvx*vx^2+Fx;

%dynamics
dZdt=[vx*cos(psi)-vy*sin(psi);...
    vx*sin(psi)+vy*cos(psi);...
    w;...
    1/m*(R-Fyf*sin(gamma))+vy*w;
    kwx*(wxdes-wx);...
    1/m*(Fyr+Fyf*cos(gamma))-w*vx;...
    1/Jg*(-d*Fyr+(l-d)*Fyf*cos(gamma)+Mf+Mr);...
    vx0/l*(l*cos(psid)-d*sin(psid)*tan(gamma0));...
    vx0/l*(l*sin(psid)+d*cos(psid)*tan(gamma0));...
    vx0/l*tan(gamma0)];

end

