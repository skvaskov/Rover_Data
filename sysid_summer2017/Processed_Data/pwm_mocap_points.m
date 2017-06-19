function [P,vx,ax] = pwm_mocap_points(data,tnums,threshold,pl)
%input:structure of data 
%trialnums: numbers in data structure to use
%threshold: minimum velocity for data points
%pl 1 if you want to plot data
%output ax: acceleration, vx: velocity, P: throttle channel in colum
%vectors
ax=[];
vx=[];
P=[];
for i=1:length(tnums)

idxs=find(data(tnums(i)).interp.mocap.speed_smooth>=threshold,1);
idxe=find(data(tnums(i)).interp.mocap.speed_smooth>=threshold,1,'last');

ax=[ax;data(tnums(i)).interp.mocap.longaccel_smooth(idxs:idxe)];
vx=[vx;data(tnums(i)).interp.mocap.speed_smooth(idxs:idxe)];
P=[P;data(tnums(i)).interp.input.throttle(idxs:idxe)];
    
if pl
figure(1)
hold on
plot(data(tnums(i)).interp.time(idxs:idxe),data(tnums(i)).interp.input.throttle(idxs:idxe))
title('trained inputs')
end
end


end

