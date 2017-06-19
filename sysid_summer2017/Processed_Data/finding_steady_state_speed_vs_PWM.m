clear ; clc ; close all
load('throttle_6_1.mat')

% matrices to be filled in by the loop:
tspan_mocap = nan(26,2) ;
tspan_enc = nan(26,2) ;
throttle_in = nan(26,1) ;
v_mocap = nan(26,1) ;
v_enc = nan(26,2) ;

% for each trial...
for idx = [1:7,9:26]
    % input for the current trial
    throttle_in(idx) = max(processeddata(idx).input.throttle(:)) ;
    
    % plot the mocap data and have the user enter start and end times
    plot(processeddata(idx).mocap.time,processeddata(idx).mocap.speed)
    grid on
    
    tlo = input(['Trial ',num2str(idx),' mocap t start: ']) ;
    thi = input(['Trial ',num2str(idx),' mocap t end: ']) ;
    
    tspan_mocap(idx,:) = [tlo,thi] ;
    t_idx = (processeddata(idx).mocap.time >= tlo) & (processeddata(idx).mocap.time <= thi) ;
    v_mocap(idx) = mean(processeddata(idx).mocap.speed(t_idx)) ;
    
    cla
    
    % plot the encoder data and have the user enter tlo and thi
    plot(processeddata(idx).encoder.time,processeddata(idx).encoder.left,...
         processeddata(idx).encoder.time,processeddata(idx).encoder.right)
    
    grid on
    
    tlo = input(['Trial ',num2str(idx),' encoder t start: ']) ;
    thi = input(['Trial ',num2str(idx),' encoder t end: ']) ;
    
    tspan_enc(idx,:) = [tlo,thi] ;
    t_idx = (processeddata(idx).encoder.time >= tlo) & (processeddata(idx).encoder.time <= thi) ;
    v_enc(idx,:) = [mean(processeddata(idx).encoder.left(t_idx)), ...
                    mean(processeddata(idx).encoder.right(t_idx))] ;
                  
	cla
end

close all

%% plotting results
[throttle_sort,sort_idx] = sort(throttle_in) ;
 
plot(throttle_sort,v_mocap(sort_idx),'b*--','LineWidth',1.25) ; hold on
plot(throttle_sort,v_enc(sort_idx,1),'r*--','LineWidth',1.25) ;
plot(throttle_sort,v_enc(sort_idx,2),'g*--','LineWidth',1.25) ;
legend('mocap','left enc','right enc','Location','best')
set(gca,'FontSize',15)

xlabel('PWM Command') ; ylabel('Speed [m/s]')