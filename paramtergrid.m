clear all
clc
%grid parameter space for pajecka model to try and get a reasonable initial
%guess with the other parameters defined

datastruct = load('reprocessedmarchlinedup.mat') ;
tarray=[10,17];

%define step of trial to look at and set up to simulate
start=25;
finish=150;
tvec=zeros(1,finish-start+1,length(tarray));
uvec=zeros(2,finish-start+1,length(tarray));
xstates=zeros(5,finish-start+1,length(tarray));
for i=1:length(tarray)
    trial= datastruct.(['processeddata',num2str(tarray(i))]);
    trial=trial(:,start:finish);
    tvec(1,:,i)=trial(1,:)-trial(1,1);
    uvec(:,:,i)=[trial(32,:);trial(8,:)];
    xstates(:,:,i)=trial([2 3 20 10 26],:);
end

fdyn=@lygerosMagic;
%enter closely known paramters
m=2.759;%mass
lf=2;
l=2.9;
p1=-1.15;
p2=0.6;
%define guesses for moment of inertia (somewhat known)
Izg=logspace(log10(.25),1,50);%moment of inertia

%define lower and upper bounds for Bf,Cf,Df
lb=[0,0,0];
ub=[100,100,100];
%number of steps
dp=100;

% Bf=logspace(log10(lb(1)),log10(ub(1)),dp);
% Cf=logspace(log10(lb(2)),log10(ub(2)),dp);
% Df=logspace(log10(lb(3)),log10(ub(3)),dp);

Bf=linspace(lb(1),ub(1),dp);
Cf=linspace(lb(2),ub(2),dp);
Df=linspace(lb(3),ub(3),dp);

%store best guess for each moment of ineritia guess (Bf,Cf,Df,cost)
bestguess=[Izg',zeros(length(Izg),3),1000*ones(length(Izg),1)];
cost=0;


for mi=1:length(Izg) 
  
for i=1:length(Bf)
     for j=1:length(Cf)
         for k=1:length(Df)
            %simulate dynamics with guess
            %p = [m,Iz,lf,l,Bf,Cf,Df,p1,p2]';
            p=[m,Izg(mi),lf,l,Bf(i),Cf(j),Df(k),p1,p2];
            cost=0;
            for ci=1:length(tarray)
            [simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec(1,:,ci),uvec(:,:,ci),xstates(:,1,ci),p);
            %calculate error from trial
            diff=simx(:,:)-xstates(:,:,ci);
            diff=diff(:);
            cost=cost+sum(diff.^2);
            end
            
            if cost<bestguess(mi,5)
                bestguess(mi,:)=[Izg(mi),Bf(i),Cf(j),Df(k),cost];
            end
         end
     end
end
 
   disp(['Best guess: ',mat2str(bestguess(mi,:))])
end

disp('Done')

