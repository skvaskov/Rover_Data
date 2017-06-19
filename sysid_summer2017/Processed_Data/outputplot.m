function outputplot(data,num)
%generate plots for processeddata
time=data(num).interp.time;

figure
subplot(4,2,1)
hold on
plot(data(num).interp.mocap.pos(1,1),data(num).interp.mocap.pos(1,2),'b*')
plot(data(num).interp.mocap.pos(end,1),data(num).interp.mocap.pos(end,2),'r*')
plot(data(num).interp.mocap.pos(:,1),data(num).interp.mocap.pos(:,2),'g')
legend('Start','Finish')
ylabel('Y position')
xlabel('X position')
hold off

subplot(4,2,2)
hold on
plot(time,data(num).interp.encoder.left)
plot(time,data(num).interp.encoder.left_smooth)
plot(time,data(num).interp.encoder.right)
plot(time,data(num).interp.encoder.right_smooth)
plot(time,data(num).interp.mocap.longvelocity_smooth)
ylabel('Encoderdata')
xlabel('Time')
legend('Left','Left Smooth', 'Right','Right Smooth','LongV Smooth')

subplot(4,2,3)
plot(time,data(num).interp.mocap.heading,'b')
hold on
plot(time,data(num).interp.mocap.heading_unwrapped_smooth,'r')
%plot(time,data(num).interp.imu.heading,'g')
%plot(time,data(num).interp.imu.heading_unwrapped_smooth,'m')
%plot(time,data(num).interp.mocap.bodyslip,'g')
%plot(time,data(num).interp.mocap.bodyslip_smooth,'m')
legend('calculated','unwrapped & smoothed','calculatedimu','unwrapped & smoothed imu','sideslip','sideslip smooth')
ylabel('heading')
xlabel('Time')
hold off

subplot(4,2,4)
plot(time,data(num).interp.mocap.speed,'k')
hold on
plot(time,data(num).interp.mocap.speed_smooth,'c')
plot(time,data(num).interp.mocap.longvelocity,'b')
plot(time,data(num).interp.mocap.longvelocity_smooth,'r')
plot(time,data(num).interp.mocap.latvelocity,'g')
plot(time,data(num).interp.mocap.latvelocity_smooth,'m')
ylabel('Veocity')
xlabel('Time')
legend('speed','speed smooth','long v', 'long v smooth','lat v', 'lat v smooth')
hold off

subplot(4,2,5)
plot(time,data(num).interp.mocap.longaccel,'b')
hold on
plot(time,data(num).interp.mocap.longaccel_smooth,'r')
plot(time,data(num).interp.mocap.lataccel,'g')
plot(time,data(num).interp.mocap.lataccel_smooth,'m')
legend('long','long smooth', 'lat', 'lat smooth')
ylabel('Acceleration (Mocap)')
xlabel('Time')

hold off

subplot(4,2,6)
plot(time,data(num).interp.imu.longaccel,'b')
hold on
plot(time,data(num).interp.imu.longaccel_smooth,'r')
plot(time,data(num).interp.imu.lataccel,'g')
plot(time,data(num).interp.imu.lataccel_smooth,'m')
legend('long','long smooth', 'lat', 'lat smooth')
ylabel('Acceleration (imu)')
xlabel('Time')
hold off

subplot(4,2,7)
hold on
plot(time,data(num).interp.mocap.yawrate,'b')
plot(time,data(num).interp.mocap.yawrate_smooth,'r')
plot(time,data(num).interp.imu.yawrate,'g')
plot(time,data(num).interp.imu.yawrate_smooth,'m')
%plot(time,data(num).interp.mocap.bodyslip_dt,'k')
%plot(time,data(num).interp.mocap.bodyslip_dt_smooth,'c')
xlabel('Time')
ylabel('Yaw Rate ')
legend('mocap', 'mocap smooth', 'imu', 'imu smooth','sideslipdt','sideslipdt_smooth')


hold off

subplot(4,2,8)
plot(time,data(num).interp.input.throttle,'b')
hold on
plot(time,data(num).interp.input.steering,'g')
%plot(time,throttlercIS,'r')
%plot(time,steeringrcIS,'m')
ylabel('Inputs')
xlabel('Time')
legend('throttle', 'steering')
hold off



end

