clear all
clc

datastruct = load('reprocessedmarchlinedup.mat') ;
tarray=[1,3,5,7,9,10,14,17,20,22,25];
data = {} ;
input = {} ;
t = {} ;
start=25;
done=100;

%time delay (number of steps)

for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
    data{idx} = trial([2,3,20,8,10,26],start:done);
    input{idx} = [steeringmodel(trial(32,start:done));trial(30,start:done)];
    t{idx} = trial(1,start:done); 
end

%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: .1585

fdyn = @fullbikeraj;
%p = [m,Iz,lf,l,ca,a,b,c,kp,ps]';
p0=  [2.75900045067479;1.00098704870458;1.58500001838122;2.99999999088277;1.30365940299509;...
    3.4803143019985;4.65090525170189;0.630246867579931;8.55671242168168;4.77656215775087];

pub=[2.76,5,3,3,1e6,1e6,1e6, 1e6,    1e6,.1 ]';
plb=[2.758,1.585,2.9,0,     0, 0,   0,  0,   0  ]';

user = nonlinearModelFit(fdyn,t,data,input,p0,'pu',pub,'pl',plb,'x2track',[1,2]) ;

