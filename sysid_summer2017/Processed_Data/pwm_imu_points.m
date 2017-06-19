function [P,vx,ax] = pwm_imu_points(data,tnums,threshold,pl)
%input:structure of data 
%trialnums: numbers in data structure to use
%threshold: minimum velocity for data points

%output ax: acceleration, vx: velocity, P: throttle channel in colum
%vectors
ax=[];
vx=[];
P=[];
for i=1:length(tnums)

idxs=find(1.3/1.193*(data(tnums(i)).interp.encoder.left_smooth+data(tnums(i)).interp.encoder.right_smooth)/2>=threshold,1);
idxe=find(1.3/1.193*(data(tnums(i)).interp.encoder.left_smooth+data(tnums(i)).interp.encoder.right_smooth)/2>=threshold,1,'last');

ax=[ax;data(tnums(i)).interp.imu.longaccel(idxs:idxe)];
vx=[vx;1.3/1.193*(data(tnums(i)).interp.encoder.left_smooth(idxs:idxe)+data(tnums(i)).interp.encoder.right_smooth(idxs:idxe))/2];
P=[P;data(tnums(i)).interp.input.throttle(idxs:idxe)];

if pl
figure(1)
hold on
plot(data(tnums(i)).interp.time(idxs:idxe),data(tnums(i)).interp.input.throttle(idxs:idxe))
title('trained inputs')
end
end


end