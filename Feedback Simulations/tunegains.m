
clear all
close all
clc
%generate random desired trajectory
num=5;
gmax=5; %maximum steering angle (degrees)
vmax=1;%maximum velocity (m/s)
steps=5*ones(1,num);
garray=rand(1,num)*gmax*2*pi/180-gmax*pi/180;
vxarray=rand(1,num)*vmax;
steerfun=@(t) step(t,garray,steps);
vxfun=@(t) step(t,vxarray,steps);

tarray=cumsum(steps);
dt=1/100;
tvector=0:dt:tarray(end);
Re=sqrt(.116^2-.01^2);
l=.3;
d=.3-.1585;
Z0=[0;0;0;vxarray(1);vxarray(1)/Re;0;0;garray(1)];
Z0=[Z0;Z0(1:3)];
[tconst,zconst]=ode45(@(t,Z) bikeconstv(t,Z,steerfun,vxfun,[l;d]),tvector,Z0(1:3));
vx0=vxfun(tconst);
gamma0=steerfun(tconst);

t{1}=tconst';
data{1}=[zconst';vx0';zeros(3,length(tconst));gamma0'];
input{1}=[gamma0';vx0';zconst'];

fdyn=@bike_lineartire_gaintuning;

p0=ones(4,1);
user=nonlinearModelFit(fdyn,t,data,input,p0,'x2track',[1 2 3 4 8]);
%%
tg=0;
if tg
disp('Testing numeric vs analytic gradient')
v = rand(size(user.z0));

tic
geqNum = numericJacobian(@user.equalityConstraints,1,v)' ;
[~,geqAnl] = user.equalityConstraints(v) ;
toc

disp(['Numeric vs. Analytic Gradients: ', num2str(norm(geqAnl - geqNum))])
end
%%
user.verbose=1;
tic
[sol,prob] = user.modelFit() ;
toc

disp(['Param fit: ', mat2str(sol.p)])
disp(['Grad norm: ', num2str(norm(sol.grad))])
disp(['Hess norm: ', num2str(norm(sol.hessian))])