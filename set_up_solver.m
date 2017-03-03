clear
clc

datastruct = load('smoothdata100imutime.mat') ;
tarray=[22,17];
data = {} ;
input = {} ;
t = {} ;
start=1;
done=150;



for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
    data{idx} = trial([2,3,20,10,26],start:done);
    input{idx} = [steeringmodel(trial(32,start:done));trial(8,start:done)];
    t{idx} = trial(1,start:done); 
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

%p0=[m,1.5,1.9,2.9,1.5,1]';
p0=[2.75910149211392;1.70118417936534;1.58507316334463;2.99992456715352;1.4076866985652;1.19863040386614];
pub=[2.762,3000,2.8,3,10000,1.2]';
plb=[2.756,0,1.585,2.8,0,.8]';
scale=[1/10;1/10;1/10;1;1];


user = nonlinearModelFit(fdyn,t,data,input,p0) ;

