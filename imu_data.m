function [time,x,y,psi,vx,vy,w,throttle,steering] = imu_data(data,tnum,threshold)
%input:structure of data 
%trialnums: numbers in data structure to use
%threshold: minimum velocity for data points

%output ax: acceleration, vx: velocity, P: throttle channel in colum
%vectors

idxs=find(1.3/1.193*(data(tnum).interp.encoder.left_smooth+data(tnum).interp.encoder.right_smooth)/2>=threshold,1);
idxe=find(1.3/1.193*(data(tnum).interp.encoder.left_smooth+data(tnum).interp.encoder.right_smooth)/2>=threshold,1,'last');

time=data(tnum).interp.time(idxs:idxe);
x=data(tnum).interp.mocap.pos(idxs:idxe,1);
y=data(tnum).interp.mocap.pos(idxs:idxe,2);
psi=data(tnum).interp.imu.heading_smooth(idxs:idxe);
vx=1.3/1.193*(data(tnum).interp.encoder.left_smooth(idxs:idxe)+data(tnum).interp.encoder.right_smooth(idxs:idxe))/2;
vy=data(tnum).interp.mocap.latvelocity_smooth(idxs:idxe);
w=data(tnum).interp.imu.yawrate_smooth(idxs:idxe);
throttle=data(tnum).interp.input.throttle(idxs:idxe);
steering=data(tnum).interp.input.steering(idxs:idxe);



end