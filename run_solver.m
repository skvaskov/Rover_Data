user.verbose=1;
tic
[sol,prob] = user.modelFit() ;
toc

disp(['Param fit: ', mat2str(sol.p)])
disp(['Grad norm: ', num2str(norm(sol.grad))])
disp(['Hess norm: ', num2str(norm(sol.hessian))])

%%
load('smoothdata100imutime.mat')
trialplot=processeddata22;
start=1;
done=65;
tvec=trialplot(1,start:done)-trialplot(1,start);
uvec=[steeringmodel(trialplot(32,start:done));trialplot(8,start:done)];
x0=trialplot([2,3,20,10,26],start);
p=sol.p;
[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);
 figure
subplot(4,1,1)
plot(simx(1,:),simx(2,:))
hold on
plot(trialplot(2,:),trialplot(3,:))
xlabel('X')
ylabel('Y')
legend('simulated','actual')

subplot(4,1,2)
plot(tvec,simx(3,:))
hold on
plot(trialplot(1,:),trialplot(20,:))
xlabel('time (s)')
ylabel('heading')
legend('simulated','actual')

subplot(4,1,3)
plot(tvec,simx(4,:))
hold on
plot(trialplot(1,:),trialplot(10,:))
xlabel('time (s)')
ylabel('lat v')
legend('simulated','actual')

subplot(4,1,4)
plot(tvec,simx(5,:))
hold on
plot(trialplot(1,:),trialplot(26,:))
xlabel('time (s)')
ylabel('yaw rate')
legend('simulated','actual')


