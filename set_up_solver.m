clear
clc

datastruct = load('smoothdata100imutime.mat') ;
tarray=[17];
data = {} ;
input = {} ;
t = {} ;
start=1;
done=200;

for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
    data{idx} = trial([2,3,20],start:done);
    input{idx} = [steeringmodel(trial(32,(start:done))-40);trial(8,start:done)];
    t{idx} = trial(1,start:done)-trial(1,start) ; 
end

%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: 15.85

fdyn = @kinematicbicycle;


p0 =[.18,.29]';
plb=[.1585,.25]';
pub=[.29,.35]';
scale=[1/10;1;1];

user = nonlinearModelFit(fdyn,t,data,input,p0,'pl',plb,'pu',pub,'x2track',3) ;

