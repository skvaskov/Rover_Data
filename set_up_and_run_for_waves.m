clear all
clc

datastruct = load('smoothdata100imutime.mat') ;
tarray=[1,2,3];
data = {} ;
input = {} ;
t = {} ;
start=1;
done=150;

for idx = 1:length(tarray)
    trial = datastruct.(['crop_' num2str(tarray(idx))]) ;
    data{idx} = trial([2,3,20,10,26],:);
    input{idx} = [steeringmodel(trial(32,:));trial(8,:)];
    t{idx} = trial(1,:);
end

fdyn = @lygerostan;

cag=[.5,2,10,15,.5,2,10,15];
Izg=[.5,.5,.5,.5,1,1,1,1];
pub=[2.762,3000,2.8,3,100000,1.5]';
plb=[2.756,0,1.585,2.8,0,.5]';
psol=zeros(length(cag),6);
xsol=zeros(5,length(tarray)*(done-start+1),length(cag));
optsol=zeros(length(cag),2);
tic

for i=1:length(cag)
    
p0=[2.759,Izg(i),1.8,2.9,cag(i),1]';
user = nonlinearModelFit(fdyn,t,data,input,p0,'pl',plb,'pu',pub) ;
user.verbose=1;
[sol,~] = user.modelFit() ;
xsol(:,:,i)=sol.x;
psol(i,:)=sol.p;
optsol(i,:)=[sol.output.firstorderopt, sol.finalCost];
end
toc
clearvars -except xsol psol optsol
save(['sol.mat'])


