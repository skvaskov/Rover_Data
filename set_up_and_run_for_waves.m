clear
clc

datastruct = load('smoothdata100imutime.mat') ;
tarray=[17 22 25 10];
data = {} ;
input = {} ;
t = {} ;
start=1;
done=150;

for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
    data{idx} = trial([2,3,20,10,26],start:done);
    input{idx} = [steeringmodel(trial(32,(start:done))-40);trial(8,start:done)];
    t{idx} = trial(1,start:done)-trial(1,start) ; 
end

fdyn = @lygerostan;
cag=[1 5 10 50 100 1000];
pub=[2.762,300,2.8,3,100000,1.5]';
plb=[2.756,0,1.585,2.8,0,.5]';

tic
for i=1:length(cag)

p0=[2.759,1.5,.18,.29,cag(i),1]';
user = nonlinearModelFit(fdyn,t,data,input,p0,'pl',plb,'pu',pub) ;
[sol(i),~] = user.modelFit(2) ;

end
toc