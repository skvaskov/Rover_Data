clear all
clc
%grid parameter space for pajecka model to try and get a reasonable initial
%guess with the other parameters defined

datastruct = load('reprocessedmarchlinedup.mat') ;
trial= datastruct.('processeddata17');
%define step of trial to look at and set up to simulate
start=25;
finish=150;
trial=trial(:,start:finish);
tvec=trial(1,:)-trial(1,1);
uvec=[trial(32,:);trial(8,:)];
fdyn=@lygerosMagic;
xstates=trial([2 3 20 10 26],:);
x0=xstates(:,1);
%enter closely known paramters
m=2.759;%mass
lf=2;
l=2.9;
p1=-1.25;
p2=0;
%define guesses for moment of inertia (somewhat known)
Izg=linspace(.25,10,100);%moment of inertia

%define lower and upper bounds for Bf,Cf,Df
lb=[10,.01,.1];
ub=[5000,100,5000];
%number of steps
dp=1000;

Bf=logspace(log10(lb(1)),log10(ub(1)),dp);
Cf=logspace(log10(lb(2)),log10(ub(2)),dp);
Df=logspace(log10(lb(3)),log10(ub(3)),dp);

%store best guess for each moment of ineritia guess (Bf,Cf,Df,cost)
bestguess=[Izg',zeros(length(Izg),3),1000*ones(length(Izg),1)];
cost=0;


for mi=1:length(Izg) 
    count=1;
for i=1:length(Bf)
     for j=1:length(Cf)
         for k=1:length(Df)
            %simulate dynamics with guess
            %p = [m,Iz,lf,l,Bf,Cf,Df,p1,p2]';
            p=[m,Izg(mi),lf,l,Bf(i),Cf(j),Df(k),p1,p2];
            [simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);
            
            %calculate error from trial
            cost=sum((simx(:)-xstates(:)).^2);
            if cost<bestguess(mi,5)
                bestguess(mi,:)=[Izg(mi),Bf(i),Cf(j),Df(k),cost];
            end
            
         end
     end
 end
   disp(['Best guess: ',mat2str(bestguess(mi,:))])
end


