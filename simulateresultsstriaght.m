%simulate results for random inputs
load('smoothdata100imutime.mat')

trial=processeddata2;
start=1;
fin=250;
tvec=trial(1,start:fin)-trial(1,start);

uvec=voltageModel(trial(30,start:fin));

tau=.5;
k=.5;
p=[tau;k];


fdyn=@gain;

x0=[trial([4,6],start)];

[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);

figure
subplot(3,1,1)
plot(tvec,simx(1,:))
hold on
plot(tvec,trial(4,start:fin))
xlabel('time (s)')
ylabel('distance')
legend('simulated','actual')

subplot(3,1,2)
plot(tvec,simx(2,:))
hold on
plot(tvec,trial(6,start:fin))
xlabel('time (s)')
ylabel('speed')
legend('simulated','actual')


subplot(3,1,3)
plot(tvec,uvec)
xlabel('time (s)')
ylabel('input')


% 