clear all
clc

datastruct = load('t17.mat') ;
tarray=[17];
data = {} ;
input = {} ;
t = {} ;
start=25;
done=150;

%time delay (number of steps)

tdelay=0;
for idx = 1:length(tarray)
    %trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
    trial=datastruct.('lineduptimepsi23');
    data{idx} = trial([2,3,20,10,26],start:done);
    input{idx} = [steeringmodel(trial(32,start-tdelay:done-tdelay));trial(8,start:done)];
    t{idx} = trial(1,start:done); 
end

%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: .1585

fdyn = @lygerosMagic;
p0=[2.759,.5,2,2.9,1,1,1,1,1,1,1,0]';

  pub=[2.762,50,3.4,3,10000*ones(1,6),1.2,.1]';
    plb=[2.755,0,1.585,2.8,-10000*ones(1,6),.8,-.1]';

user = nonlinearModelFit(fdyn,t,data,input,p0,'pu',pub,'pl',plb) ;

