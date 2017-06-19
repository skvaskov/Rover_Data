% clear all
% close all
% load('motor_pwm.mat')
contrval=[1580,1540,1500,1480,1440,1420,1500,1400,1380,1360,1340,1320,1490,1640,1620,1600];
contrtime=[5,5,5,5,5,6,5,5,5,5,5,5,5,2,2.5,4];
for i=2:17
    idx0=find(data(i).no.imu.time>0,1)-1;
    data(i).no.imu.throttle=zeros(size(data(i).no.imu.time));
    idxsp=find(data(i).no.imu.time<=1,1,'last');
    data(i).no.imu.throttle(idx0:idxsp)=1660*ones(1,idxsp-idx0+1);
    idxend=find(data(i).no.imu.time<=1+contrtime(i-1),1,'last');
    data(i).no.imu.throttle(idxsp+1:idxend)=contrval(i-1)*ones(1,idxend-idxsp);
end

    