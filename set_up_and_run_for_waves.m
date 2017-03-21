clear all
clc

datastruct = load('reprocessedmarchlinedup.mat') ;
tarray=[17,10,14,20,22,25];
data = {} ;
input = {} ;
t = {} ;
start=25;
done=50;

for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
    data{idx} = trial([2,3,20,10,26],start:done);
    input{idx} = [steeringmodel(trial(32,start:done));trial(8,start:done)];
    t{idx} = trial(1,start:done);
end

fdyn = @lygerosMagic;


    
guess(1,:)=[0.5, 1.32194114846603e-05, 50, 780];
guess(2,:)=[1.33333333333333, 1.51991108295293e-05, 110, 950];
guess(3,:)=[2.16666666666667, 1.32194114846603e-05, 240, 850];
guess(4,:)=[3, 0.00028480358684358, 20, 670];
guess(5,:)=[3.83333333333333, 1e-05, 520, 950];
guess(6,:)=[4.66666666666667, 5.33669923120631e-05, 160, 710];
guess(7,:)=[5.5, 3.05385550883341e-05, 370, 630];
guess(8,:)=[6.33333333333333, 1.51991108295293e-05,540, 940];
guess(9,:)=[7.16666666666667, 2.31012970008316e-05, 410, 840];
guess(10,:)=[8, 1.74752840000768e-05, 520, 890];
pub=[2.76,9,2.8,3,1000,1000,1000]';
plb=[2.758,.4,1.585,2.8,0,0,0]';

tic
sz=size(guess);
results=zeros(sz(1),9);

for i=1:sz(1)
    
p0=[2.759,guess(i,1),2,2.9,guess(i,2),guess(i,3),guess(i,4)]';
user = nonlinearModelFit(fdyn,t,data,input,p0,'pl',plb,'pu',pub) ;
user.verbose=1;
[sol,~] = user.modelFit() ;
results(i,:)=[sol.p',sol.finalCost,sol.output.firstorderopt];

end

toc
disp('Done. Now check your results')


