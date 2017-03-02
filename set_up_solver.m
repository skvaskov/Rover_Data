clear
clc

datastruct = load('smoothdata100imutime.mat') ;
tarray=1:9;
data = {} ;
input = {} ;
t = {} ;
start=1;
done=50;



for idx = 1:length(tarray)
    trial = datastruct.(['crop_' num2str(tarray(idx))]) ;
    data{idx} = trial([2,3,20,10,26],1:10);
    input{idx} = [steeringmodel(trial(32,1:10));trial(8,1:10)];
    t{idx} = trial(1,1:10); 
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

%p0=[m,1,1.9,2.9,.5,1]';
p0=[2.75899704295464;0.938431102280719;1.63937194506105;2.84198638881037;0.561753788680938;0.93700027118398];

pub=[2.762,3000,2.8,3,10000,1.5]';
plb=[2.756,0,1.585,2.8,0,.5]';
scale=[1/10;1/10;1/10;1;1];


user = nonlinearModelFit(fdyn,t,data,input,p0,'pl',plb,'pu',pub) ;

