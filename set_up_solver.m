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
    data{idx} = trial([2,3,20,10,26],start:done);
    input{idx} = [steeringmodel(trial(32,(start:done))-40);trial(8,start:done)];
    t{idx} = trial(1,start:done)-trial(1,start) ; 
end

%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: .1585

fdyn = @lygerostan;

l=.29;
lf=.18;
m=2.759;
w=.3;
Izub=2.759/12*(l^2+w^2);
Izg=2.759/12*l^2;
lflb=.1585;

p0=[m,Izg,lf,l,1,1]';
pub=[2.762,3,.28,.3,10000,1.5]';
plb=[2.756,0,.1585,.28,0,.5]';
scale=[1/10;1/10;1/10;1;1];


user = nonlinearModelFit(fdyn,t,data,input,p0,'pl',plb,'pu',pub,'x2track',3) ;

