clear all
datastruct = load('smoothdata100post.mat');

trialnums=[1 4 6 25];
rows=[4 6 29];
tc=['r' 'b' 'g' 'k'];

for i=1:length(trialnums)
   
    index=trialnums(i);
    trial = datastruct.(['processeddata' num2str(index)]);
    
  subplot(3,1,1)
    plot(trial(1,:),trial(rows(1),:),tc(i))
    hold on
    
    
subplot(3,1,2)
plot(trial(1,:),trial(rows(2),:),tc(i))
hold on

subplot(3,1,3)
plot(trial(1,:),trial(rows(3),:),tc(i))
hold on

end
subplot(3,1,1)
  xlabel('X(m)')
    ylabel('Y(m)')
    legend(['trial ' num2str(trialnums(1))],['trial ' num2str(trialnums(2))],['trial ' num2str(trialnums(3))],['trial ' num2str(trialnums(4))])
    title('Trials 17,18,19')
    
    
    subplot(3,1,2)
    xlabel('Time (s)')
ylabel('heading (rad)')
    legend(['trial ' num2str(trialnums(1))],['trial ' num2str(trialnums(2))],['trial ' num2str(trialnums(3))],['trial ' num2str(trialnums(4))])


 subplot(3,1,3)
    xlabel('Time (s)')
ylabel('steering')
    legend(['trial ' num2str(trialnums(1))],['trial ' num2str(trialnums(2))],['trial ' num2str(trialnums(3))],['trial ' num2str(trialnums(4))])
