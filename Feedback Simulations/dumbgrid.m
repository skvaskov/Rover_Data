%solve for gains with low cost by gridding parameter space
%define variable gains
kx=logspace(-1,2,5);
ky=logspace(-1,2,5);
kpsi=logspace(-1,2,5);
kvx=logspace(-1,2,5);

%define Parameters
m=2.759;
Jg=.1733;
l=.3;
d=.3-.1585;
ca=24;
Re=sqrt(.116^2-.01^2);
a=0;
b=0;
c=0;
kwx=.85;
kgamma=10;

%enter desired steering and velocity inputs (for trajectory to follow)
num=10;
gmax=5; %maximum steering angle (degrees)
vmax=1;%maximum velocity (m/s)
steps=5*ones(1,num);
garray=rand(1,num)*gmax*2*pi/180-gmax*pi/180;
vxarray=rand(1,num)*vmax;
steerfun=@(t) step(t,garray,steps);
vxfun=@(t) step(t,vxarray,steps);

%define initial condition
%simulate results
tarray=cumsum(steps);
tspan=[0,tarray(end)];
Z0=[0;0;0;vxarray(1);vxarray(1)/Re;0;0;garray(1)];
Z0=[Z0;Z0(1:3)];
cost=1e6;
Gains=[ky(1) kpsi(1) kx(1) kvx(1)];
disp('Start')
for iy=1:length(ky)
    for ipsi=1:length(kpsi)
        for ix=1:length(kx)
            for ivx=1:length(kvx)
               P=[m,Jg,l,d,ca, Re, a, b,c,ky(iy),kpsi(ipsi),kx(ix),kvx(ivx),kwx,kgamma];
               [tfb,zfb]=ode45(@(t,Z) bike_lineartire_wx_feedback(t,Z,steerfun,vxfun,P),tspan,Z0);
               diff=zfb(:,1:3)-zfb(:,9:11);
               c=sum(diff(:).^2);
               if c<cost
                   cost=c;
                   Gains=[ky(iy),kpsi(ipsi),kx(ix),kvx(ivx)];
                   disp(['Gains (ky kpsi kx kvx):' mat2str(Gains)])
                    disp(['Cost: ' num2str(cost)])
               end
            end
        end
    end
end
disp('Done')
disp(['Best Gains (ky kpsi kx kvx):' mat2str(Gains)])
disp(['Cost: ' num2str(cost)])


               
