%compare simulations of fitted trials
clear all
datastruct = load('smoothdata100post.mat');

trialnums=[11 16 19 22 23];
rows=[22 26 32];
tc=['r' 'b' 'g' 'k' 'm'];
range=200;
fdyn=@slipyaw;
start=25;

p=[1.35085963779836;0.0135116316691502;0.0518317062864032;0.18145837976221;5.7136065689714;0.113087860372008];
for i=1:length(trialnums)
    
    index=trialnums(i);
    trial = datastruct.(['processeddata' num2str(index)]) ;
    
    tvec=trial(1,start:range);
    uvec=[1/100*trial(32,start:range);trial(8,start:range)];
    x0=trial([22 26],start);
    [simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);
    
    subplot(3,1,1)
    plot(tvec,simx(1,:),tc(i))
      hold on
    plot(trial(1,start:range),trial(22,start:range),[tc(i) '--'])
  
    
    subplot(3,1,2)
    plot(tvec,simx(2,:),tc(i))
    hold on
    plot(trial(1,start:range),trial(26,start:range),[tc(i) '--'])
    
    subplot(3,1,3)
    plot(tvec,uvec(1,:),tc(i))
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