clear all
clc
load('results_3_21_17.mat')
datastruct = load('reprocessedmarchlinedup.mat') ;
tarray=[17,10,14,20,22,25];
data = {} ;
input = {} ;
t = {} ;
start=25;
done=100;

for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
    data{idx} = trial([2,3,20,10,26],start:done);
    input{idx} = [steeringmodel(trial(32,start:done));trial(8,start:done)];
    t{idx} = trial(1,start:done);
end

fdyn = @lygerosraj;


    
guess=[.5,.5;
        .5,1;...
        1,.5;...
        1,1;...
        2.5,1;...
        2.5,10;...
        5,1;...
        5,10];
        
    
pub=[2.76,9,2.8,3,1000]';
plb=[2.758,.4,1.585,2.8,0]';

tic
sz=size(guess);
results=zeros(sz(1),7);

for i=1:sz(1)
    
p0=[2.759,guess(i,1),2,2.9,guess(i,2)]';
user = nonlinearModelFit(fdyn,t,data,input,p0,'pl',plb,'pu',pub) ;
user.verbose=1;
[sol,~] = user.modelFit() ;
results(i,:)=[sol.p',sol.finalCost,sol.output.firstorderopt];

end

toc
disp('Done. Now check your results')


