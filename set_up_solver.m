clear
clc

datastruct = load('smoothdata100imutime.mat') ;
tarray=[22,17,10];
data = {} ;
input = {} ;
t = {} ;
start=25;
done=75;



for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
    data{idx} = trial([2,3,20,10,26],start:done);
    input{idx} = [steeringmodel(trial(32,start:done));trial(8,start:done)];
    t{idx} = trial(1,start:done); 
end

%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: .1585

fdyn = @lygerosMagic;

l=.29;
lf=.18;
m=2.759;
w=.3;
Izub=2.759/12*(l^2+w^2);
Izg=2.759/12*l^2;
lflb=.1585;

%p0=[m,1.5,1.9,2.9,1.5,1]';
p0=[2.75910149211392;1.70118417936534;2.5;2.99992456715352;1;1;1;1;1;1] ;
pub=[2.762,10,2.8,3.1,2,2,10,1.5,1.5,1.5]';
plb=[2.756,0,1.585,2.8,0,0,0,.5,.5,.5]';
scale=[1/10;1/10;1/10;1;1];


user = nonlinearModelFit(fdyn,t,data,input,p0,'pl',plb,'pu',pub) ;

