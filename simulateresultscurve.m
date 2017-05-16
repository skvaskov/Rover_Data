clear
close all
%simulate results for random inputs
datastruct = load('reprocessedmarchlinedup.mat') ;

%load('results_3_23_17.mat')






tarray=[11,14,5,24];

carray=['r','b','k','c','m'];


p=[2.759,.173344,.1585,.3,23.6796,3.3984,.00000147546,0.506742,0.371272,0.0999996]';


start=25;
fin=100;

fdyn=@fullbikeraj;
figure
for i=1:length(tarray)
trial= datastruct.(['processeddata',num2str(tarray(i))]) ;

tvec=trial(1,start:fin)-trial(1,start);

uvec=[steeringmodel(trial(32,start:fin));trial(30,start:fin)];



x0=trial([2, 3, 20,8,10,26],start);

[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);


subplot(3,2,1)
plot(simx(1,:),simx(2,:),[carray(i),'--'])
hold on
plot(trial(2,start:fin),trial(3,start:fin),carray(i))

subplot(3,2,2)
plot(tvec,simx(3,:),[carray(i),'--'])
hold on
plot(tvec,trial(20,start:fin),carray(i))

subplot(3,2,3)
plot(tvec,simx(4,:),[carray(i),'--'])
hold on
plot(tvec,trial(8,start:fin),carray(i))

subplot(3,2,4)
plot(tvec,simx(5,:),[carray(i),'--'])
hold on
plot(tvec,trial(10,start:fin),carray(i))

subplot(3,2,5)
plot(tvec,simx(6,:),[carray(i),'--'])
hold on
plot(tvec,trial(26,start:fin),carray(i))

 subplot(3,2,6)
 plot(tvec,uvec(1,:),carray(i))
 hold on
 
end

subplot(3,2,1)
xlabel('X')
ylabel('Y')
legend('simulated','actual')

subplot(3,2,2)
xlabel('time (s)')
ylabel('heading')


subplot(3,2,3)
xlabel('time (s)')
ylabel('long v')


subplot(3,2,4)
xlabel('time (s)')
ylabel('lat v')


subplot(3,2,5)
xlabel('time (s)')
ylabel('yaw rate')


subplot(3,2,6)
xlable('time (s)')
ylabel('steering angle')
