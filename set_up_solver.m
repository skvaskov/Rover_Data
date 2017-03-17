clear all
clc

datastruct = load('reprocessedmarchlinedup.mat') ;
tarray=[11,10,13,17,24];
data = {} ;
input = {} ;
t = {} ;
start=25;
done=75;

%time delay (number of steps)

for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
   
    data{idx} = trial([2,3,20],start:done);
    input{idx} = [trial(31,start:done);trial(8,start:done)];
    t{idx} = trial(1,start:done); 
end

%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: .1585

fdyn = @kinematicbike;
p0=[2.9,-1.15,0.08]';

  pub=[3,-.75,1]';
   plb=[2.8,-1.75,]';

user = nonlinearModelFit(fdyn,t,data,input,p0,'pu',pub,'pl',plb) ;

