function [time,x,y,psi,vx,vy,w,throttle,steering] = mocap_data(data,tnum,threshold)
%input:structure of data 
%trialnums: numbers in data structure to use
%threshold: minimum velocity for data points

%output ax: acceleration, vx: velocity, P: throttle channel in colum
%vectors

idxs=find(data(tnum).interp.mocap.speed_smooth>=threshold,1);
idxe=find(data(tnum).interp.mocap.speed_smooth>=threshold,1,'last');

time=data(tnum).interp.time(idxs:idxe);
x=data(tnum).interp.mocap.pos(idxs:idxe,1);
y=data(tnum).interp.mocap.pos(idxs:idxe,2);
psi=data(tnum).interp.mocap.heading_unwrapped_smooth(idxs:idxe);
vx=data(tnum).interp.mocap.longvelocity_smooth(idxs:idxe);
vy=data(tnum).interp.mocap.latvelocity_smooth(idxs:idxe);
w=data(tnum).interp.mocap.yawrate_smooth(idxs:idxe);
throttle=data(tnum).interp.input.throttle(idxs:idxe);
steering=data(tnum).interp.input.steering(idxs:idxe);



end