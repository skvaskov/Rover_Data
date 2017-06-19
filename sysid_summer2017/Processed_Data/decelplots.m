%generate plots of deceleration
load('motor_pwm.mat')
speed=[15,16,17];
coast=[2,3,4,5,14];
decel=[6,7,10,11,12,13];

for i=1:length(speed)
    figure(1)
    subplot(3,2,1)
    hold on
    plot(data(speed(i)).no.mocap.time,data(speed(i)).no.mocap.longvelocity)
    
    xlim([0,5])
end

for i=1:length(coast)
    figure(1)
    subplot(3,2,3)
    hold on
    plot(data(coast(i)).no.mocap.time,data(coast(i)).no.mocap.longvelocity)
    xlim([0,5])
end
for i=1:length(decel)
     figure(1)
    subplot(3,2,5)
    hold on
    plot(data(decel(i)).no.mocap.time,data(decel(i)).no.mocap.longvelocity)
    xlim([0,5])
end