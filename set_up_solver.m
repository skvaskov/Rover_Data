clear all
clc

datastruct = load('reprocessedmarchlinedup.mat') ;
tarray=[17,10,14,20,22,25];
data = {} ;
input = {} ;
t = {} ;
start=50;
done=100;

%time delay (number of steps)

for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
   
    data{idx} = trial([2,3,20,10,26],start:done);
    input{idx} = [steeringmodel(trial(32,start:done));trial(8,start:done)];
    t{idx} = trial(1,start:done); 
end

%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: .1585

fdyn = @lygerosraj;
p0=[2.759,2.6,2.5,2.9,1]';

pub=[2.76,5,2.8,3,1000]';
plb=[2.758,.5,1.585,2.8,0]';

user = nonlinearModelFit(fdyn,t,data,input,p0,'pu',pub,'pl',plb) ;

