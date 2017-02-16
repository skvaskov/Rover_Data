%compare simulations of fitted trials
clear all
datastruct = load('smoothdata100post.mat');

trialnums=[2 3 5 7 9];
rows=[2 19 31];
tc=['r' 'b' 'g' 'k' 'm'];
range=250;
fdyn=@straightModel;

p=[2.45141230931482;-2.56900505357345;3.78395900443509;-7.23793374975934;5.18546945917286;-3.24735487027097;5.36586675239555;-4.19343611851299;-0.00565529542936282];

for i=1:length(trialnums)
    
    index=trialnums(i);
    trial = datastruct.(['processeddata' num2str(index)]) ;
    
    tvec=trial(1,1:range);
    uvec=1/100*trial(29,1:range);
    x0=trial([4 6],1);
    [simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);
    
    subplot(3,1,1)
    plot(tvec,simx(1,:),tc(i))
    plot(trial(1,1:range),trial(4,1:range),[tc(i) '--'])
    hold on
    
    subplot(3,1,2)
    plot(tvec,simx(2,:),tc(i))
    plot(trial(1,1:range),trial(6,1:range),[tc(i) '--'])
    hold on
    
    subplot(3,1,3)
    plot(tvec,uvec,tc(i))
    hold on
end

subplot(3,1,1)
xlabel('time (s)')
ylabel('distance (m)')
title('Simulated vs Actual Data')

subplot(3,1,2)
xlabel('time(s)')
ylabel('velocity m/s')

subplot(3,1,3)
xlabel('time (s)')
ylabel('Input')
legend(['Trial ' num2str(trialnums(1))], ['Trial ' num2str(trialnums(2))], ['Trial ' num2str(trialnums(3))],...
    ['Trial ' num2str(trialnums(4))], ['Trial ' num2str(trialnums(5))])