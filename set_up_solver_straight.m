clear
clc

datastruct = load('smoothdata100imutime.mat') ;
tarray=[2,4,7,9,10];
data = {} ;
input = {} ;
t = {} ;
start=1;
done=200;



for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
    data{idx} = trial([4,6],start:done);
    input{idx} = trial(30,start:done);
    t{idx} = trial(1,start:done); 
end

%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: .1585

fdyn = @gain;


tau=.5;
k=.5;
%p0=[tau;k];
p0=[0.68538290883681;0.747232091411109];


scale=[1/10;1/10;1/10;1;1];
plb=[0,0]';

user = nonlinearModelFit(fdyn,t,data,input,p0,'pl',plb) ;

