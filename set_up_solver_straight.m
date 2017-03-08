clear
clc

datastruct = load('smoothdata100imutime.mat') ;
tarray=[9,2];
data = {} ;
input = {} ;
t = {} ;
start=25;
done=100;



for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
    data{idx} = [trial([4,6],start:done);zeros(1,done-start+1)];
    input{idx} = trial(30,start:done);
    t{idx} = trial(1,start:done); 
end

%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: .1585

fdyn = @picruise;


p0=[.05;1e-6;1e-6;1.05970283987747;2.31687606793406;0.00448242517080985];

scale=[1/10;1/10;1/10;1;1];
plb=[0,0,0,0,0,0]';
pub=[10,1,1,10000,10000,1]';

user = nonlinearModelFit(fdyn,t,data,input,p0,'pl',plb,'pu',pub,'x2track',[1,2]) ;

