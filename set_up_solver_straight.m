clear all
clc

load('motor_pwm.mat') ;

statedata = {} ;
input = {} ;
t = {} ;



count=1;
for idx = 1:1:24
    [time,x,y,psi,vx,vy,w,throttle,steering] = mocap_data(data1700,idx,.2);
    statedata{count} = vx';
    input{count} = throttle';
    t{count} = time'; 
    count=count+1;
end
for idx = 1:1:12
    [time,x,y,psi,vx,vy,w,throttle,steering] = mocap_data(dataslow,idx,.2);
    statedata{count} = vx';
    input{count} = throttle';
    t{count} = time'; 
    count=count+1;
end
% for idx = 2:13
%     [time,x,y,psi,vx,vy,w,throttle,steering] = mocap_data(data,idx,.2);
%     statedata{count} = vx';
%     input{count} = throttle';
%     t{count} = time'; 
%     count=count+1;
% end
%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: .1585

fdyn = @pwmv2;


p0=[178.354733764932;-0.245372336075531;-43.5176614759968;0.0559933438760013;-0.0292264794830306;8.38258858728173e-05;-1.82431217474240e-05];

user = nonlinearModelFit(fdyn,t,statedata,input,p0) ;

