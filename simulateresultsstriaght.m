%simulate results for random inputs

load('smoothdata100imutime.mat')

trial=processeddata8;
start=1;
fin=200;
tvec=trial(1,start:fin)-trial(1,start);

uvec=trial(30,start:fin);

p=[0.05;0;0;1.05970283987747;2.31687606793406;0.00448242517080985];

fdyn=@picruise;

x0=[trial([4,6],start);0];

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