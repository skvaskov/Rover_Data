%compare controllers for accceleration
clear all
close all
clc
load('processed_control1700.mat')
trials=[7,8,9];

    
for i=1:length(trials)
    figure(1)
    subplot(length(trials),1,i)
    hold on
   	plot(processeddata(trials(i)).time,processeddata(trials(i)).mocap.longvelocity)
    plot(processeddata(trials(i)).time,processeddata(trials(i)).encoder.left)
    plot(processeddata(trials(i)).time,processeddata(trials(i)).encoder.right)
    legend('long v','left enc','right enc')
    xlabel('time')
    ylabel('velocity')
    title(trialnames(trials(i)),'interpreter','none')
end
  
    
    
    trials=[4,5,6];

for i=1:length(trials)
    figure(2)
    subplot(length(trials),1,i)
   	plot(processeddata(trials(i)).time,processeddata(trials(i)).mocap.longvelocity)
    hold on
    plot(processeddata(trials(i)).time,processeddata(trials(i)).encoder.left)
    plot(processeddata(trials(i)).time,processeddata(trials(i)).encoder.right)
    legend('long v','left enc','right enc')
    xlabel('time')
    ylabel('velocity')
    title(trialnames(trials(i)),'interpreter','none')
end
   for i=1:12
    figure(5)
    yyaxis('left')
    hold on
    plot(processeddata(i).time,processeddata(i).mocap.longvelocity,'r-')
     yyaxis('right')
     hold on
     plot(processeddata(i).time,processeddata(i).input.throttle,'g-')
end
figure(5)
  xlabel('time')
  yyaxis('left')
  ylabel('velocity')
   yyaxis('right')
   ylabel('throttle')
  title('fast controller')
%%
clear all
load('processed_controlslow.mat')
trials=[7,8,9];

    
for i=1:length(trials)
    figure(3)
    subplot(length(trials),1,i)
    hold on
   	plot(processeddata(trials(i)).time,processeddata(trials(i)).mocap.longvelocity)
    plot(processeddata(trials(i)).time,processeddata(trials(i)).encoder.left)
    plot(processeddata(trials(i)).time,processeddata(trials(i)).encoder.right)
    legend('long v','left enc','right enc')
    xlabel('time')
    ylabel('velocity')
    title(trialnames(trials(i)),'interpreter','none')
end


trials=[10,11,12];

    
for i=1:length(trials)
    figure(4)
    subplot(length(trials),1,i)
    hold on
   	plot(processeddata(trials(i)).time,processeddata(trials(i)).mocap.longvelocity)
    plot(processeddata(trials(i)).time,processeddata(trials(i)).encoder.left)
    plot(processeddata(trials(i)).time,processeddata(trials(i)).encoder.right)
    legend('long v','left enc','right enc')
    xlabel('time')
    ylabel('velocity')
    title(trialnames(trials(i)),'interpreter','none')
end
for i=10:12
    figure(6)
    yyaxis('left')
    hold on
    plot(processeddata(i).time,processeddata(i).mocap.longvelocity,'r-')
     yyaxis('right')
     hold on
    plot(processeddata(i).time,processeddata(i).input.throttle,'g-')
end
figure(6)
  xlabel('time')
  yyaxis('left')
  ylabel('velocity')
   yyaxis('right')
  ylabel('throttle')
  title('Slow Controller')
 %%
  clear all
load('processed_control1660_1620.mat')
p=zeros(5,2);
pb=zeros(5,2);
for i=1:5
    figure(7)

     yyaxis('left')
    hold on
    plot(processeddata(i).time,processeddata(i).mocap.longvelocity,'r-')
    yyaxis('right')
     hold on
    plot(processeddata(i).time,processeddata(i).input.throttle,'g-')
    %plot(processeddata(i).time,processeddata(i).mocap.pos(:,3),'b-')
    idx=find(processeddata(i).time>.5,1);
    idxbs=find(processeddata(i).time>2.6,1);
    idxbe=find(processeddata(i).time<2.8,1,'last');
    p(i,:)=polyfit(processeddata(i).time(1:idx),processeddata(i).mocap.longvelocity(1:idx),1);
    pb(i,:)=polyfit(processeddata(i).time(idxbs:idxbe),processeddata(i).mocap.longvelocity(idxbs:idxbe),1);
end

p1660=mean(p,1);
disp(['accel: ',num2str(p1660(1))])
p1400=mean(pb,1);
disp(['brake: ',num2str(p1400(1))])
t=linspace(0,.6);
tb=linspace(2.6,3.4);
figure(7)
%plot(t,polyval(p1660,t),'k')
%plot(tb,polyval(p1400,tb),'k')
 xlabel('time')
  yyaxis('left')
  ylabel('velocity')
   yyaxis('right')
  ylabel('throttle')
    