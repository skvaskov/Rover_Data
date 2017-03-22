clear
%simulate results for random inputs
datastruct = load('reprocessedmarchlinedup.mat') ;
load('results_3_21_17.mat')
tarray=[11,25];
carray=['r','b','k','c','m'];

%initial guess 2.759,1.22,1.586,2.9,.0236,7.4436,10.9012
%solved for on trials 17,10,14 steps 25-50 (first order opt e-6)
%p=[2.759,1.22,1.585,3,.0236,7.4436,10.9012]';

%solved for on trials 17,10,14,15,20,22 (first order opt e-6)
%p=[2.759,1.1098,1.585,3.000,.0093,12.1213,15.1396];

%solved for on trials 17,10,14,15,20,22 only caring about states 4,5 in cost(first order opt e-7)
%p=[2.756,.5,2,2.9,1.32e-5,50,780];
p=results(4,1:7);


start=25;
fin=50;

figure
for i=1:length(tarray)
trial= datastruct.(['processeddata',num2str(tarray(i))]) ;

tvec=trial(1,start:fin)-trial(1,start);

uvec=[steeringmodel(trial(32,start:fin));trial(8,start:fin)];

fdyn=@lygerosMagic;

x0=trial([2, 3, 20,10,26],start);

[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);


subplot(5,1,1)
plot(simx(1,:),simx(2,:),[carray(i),'--'])
hold on
plot(trial(2,start:fin),trial(3,start:fin),carray(i))

subplot(5,1,2)
plot(tvec,simx(3,:),[carray(i),'--'])
hold on
plot(tvec,trial(20,start:fin),carray(i))

subplot(5,1,3)
plot(tvec,simx(4,:),[carray(i),'--'])
hold on
plot(tvec,trial(10,start:fin),carray(i))

subplot(5,1,4)
plot(tvec,simx(5,:),[carray(i),'--'])
hold on
plot(tvec,trial(26,start:fin),carray(i))

 subplot(5,1,5)
 plot(tvec,uvec(1,:),carray(i))
 hold on
end

subplot(5,1,1)
xlabel('X')
ylabel('Y')
legend('simulated','actual')

subplot(5,1,2)
xlabel('time (s)')
ylabel('heading')
legend('simulated','actual')

subplot(5,1,3)
xlabel('time (s)')
ylabel('lat v')
legend('simulated','actual')

subplot(5,1,4)
xlabel('time (s)')
ylabel('yaw rate')
legend('simulated','actual')