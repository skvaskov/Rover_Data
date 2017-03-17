clear all
clc

datastruct = load('reprocessedmarchlinedup.mat') ;
tarray=[11,13,17,24];
data = {} ;
input = {} ;
t = {} ;
start=25;
done=75;

%time delay (number of steps)

for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
   
    data{idx} = trial([2,3,20,10,26],start:done);
    input{idx} = [trial(32,start:done);trial(8,start:done)];
    t{idx} = trial(1,start:done); 
end

%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: .1585

fdyn = @lygerosMagic;
p0=[2.759,.75,2,2.9,1.842,1.93,2.8,-1.15,0.6]';

pub=[2.76,5,2.8,3,50,10,10,-1,1]';
plb=[2.758,.5,1.585,2.8,0,0,0,-1.3,0]';

user = nonlinearModelFit(fdyn,t,data,input,p0,'pu',pub,'pl',plb) ;

