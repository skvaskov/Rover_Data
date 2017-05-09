%time
tvec=linspace(0,2);
lt=length(tvec);
%input
uvec=[.5*ones(1,50),ones(1,lt-50);...
    zeros(1,50),1.5*ones(1,lt-50)];
fdyn=@unicycle;
x0=[0;0;0];
p=0;
[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);

plot(simx(1,:),simx(2,:))