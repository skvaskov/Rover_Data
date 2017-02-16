%simulate results for random inputs
load('smoothdata100imutime.mat')

trial=processeddata17;
start=1;
fin=150;
tvec=trial(1,start:fin);
uvec=[steeringmodel(trial(32,start:fin));trial(8,start:fin)];


 l=.29;
lf=.18;
m=2.759;
w=.3;
Izub=2.759/12*(l^2+w^2);
Izg=2.759/12*l^2;
lflb=.1585;


p=[2.759,10,1.6,2.9,100,1]';

fdyn=@lygerosreduced;
x0=[trial([10,26],1)];

[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);

% subplot(4,1,1)
% plot(simx(1,:),simx(2,:))
% hold on
% plot(trial(2,:),trial(3,:))
% xlabel('X')
% ylabel('Y')
% legend('simulated','actual')
    
subplot(3,1,1)
plot(tvec,simx(1,:))
hold on
plot(trial(1,:),trial(10,:))
xlabel('time (s)')
ylabel('latv')

subplot(3,1,2)
plot(tvec,simx(2,:))
hold on
plot(trial(1,:),trial(26,:))
xlabel('time (s)')
ylabel('yawrate')


subplot(3,1,3)
plot(tvec,uvec(1,:))
hold on
plot(tvec,uvec(2,:))
ylabel('input')

% 