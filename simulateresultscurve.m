%simulate results for random inputs
load('smoothdata100imutime.mat')

trial=processeddata10;
start=1;
fin=200;
tvec=trial(1,start:fin)-trial(1,start);

uvec=[steeringmodel(trial(32,start:fin));trial(8,start:fin)];

Izg=.5;
cag=.5;

l=.29;
lf=.18;
m=2.759;
w=.3;
Izub=2.759/12*(l^2+w^2);
%Izg=2.759/12*l^2;
lflb=.1585;


%p=[m,Izg,1.8,2.9,cag,1]';
p=[3.84208168835254;0.159025121501556;0.208870239886732;2.53441240536683;1.14049824669156;0.0969432443388565];

fdyn=@lygerostan;

x0=trial([2 3 20 10 26],start);

[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);

figure
subplot(4,1,1)
plot(simx(1,:),simx(2,:))
hold on
plot(trial(2,start:fin),trial(3,start:fin))
xlabel('X')
ylabel('Y')
legend('simulated','actual')

subplot(4,1,2)
plot(tvec,simx(3,:))
hold on
plot(tvec,trial(20,start:fin))
xlabel('time (s)')
ylabel('heading')
legend('simulated','actual')

subplot(4,1,3)
plot(tvec,simx(4,:))
hold on
plot(tvec,trial(10,start:fin))
xlabel('time (s)')
ylabel('lat v')
legend('simulated','actual')

subplot(4,1,4)
plot(tvec,simx(5,:))
hold on
plot(tvec,trial(26,start:fin))
xlabel('time (s)')
ylabel('yaw rate')
legend('simulated','actual')
% 