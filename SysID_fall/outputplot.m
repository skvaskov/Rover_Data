function outputplot(trial)
%generate plots for processeddata
%plot data
%all data should be plotted against "time", which is the timesteps aligned
%with the input channels
%variableformat=datasourceXY
%data is pos,heading,acceltan,velocity,steering,or throttle
%source is mocap,imu, or rc
%X is I if the variable has been interpolated to line up "time", or nothing otherwise
%Y is S if the data has a smoothing function applied to it, or C if its
%corrected somehow
time=trial(1,:);

figure
subplot(4,2,1)
plot(time,trial(2,:),'b')
hold on
plot(time,trial(3,:),'g')
ylabel('position')
hold off

subplot(4,2,2)
plot(time,trial(4,:),'k')
ylabel('Distance')

subplot(4,2,3)
plot(time,trial(19,:),'b')
hold on
plot(time,trial(20,:),'r')
plot(time,trial(21,:),'g')
plot(time,trial(22,:),'m')
ylabel('Heading/Sideslip')
ylim([-1 1])
hold off

subplot(4,2,4)
plot(time,trial(5,:),'k')
hold on
plot(time,trial(6,:),'c')
plot(time,trial(7,:),'b')
plot(time,trial(8,:),'r')
plot(time,trial(9,:),'g')
plot(time,trial(10,:),'m')
ylabel('Veocity')
hold off

subplot(4,2,5)
plot(time,trial(15,:),'b')
hold on
plot(time,trial(16,:),'r')
plot(time,trial(17,:),'g')
plot(time,trial(18,:),'m')
ylabel('Acceleration (Mocap)')
hold off

subplot(4,2,6)
plot(time,trial(11,:),'b')
hold on
plot(time,trial(12,:),'r')
plot(time,trial(13,:),'g')
plot(time,trial(14,:),'m')
ylabel('Acceleration (imu)')
hold off

subplot(4,2,7)
plot(time,trial(27,:),'k')
hold on
plot(time,trial(28,:),'c')
plot(time,trial(25,:),'b')
plot(time,trial(26,:),'r')
plot(time,trial(23,:),'g')
plot(time,trial(24,:),'m')
xlabel('Yaw Rate/SideSlip (imu is g)')
ylim([-1.5 1.5])
hold off

subplot(4,2,8)
plot(time,trial(29,:),'b')
hold on
plot(time,trial(30,:),'r')
plot(time,trial(31,:),'g')
plot(time,trial(32,:),'m')
xlabel('Inputs')
hold off
end

