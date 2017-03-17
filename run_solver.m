user.verbose=1;
tic
[sol,prob] = user.modelFit() ;
toc

disp(['Param fit: ', mat2str(sol.p)])
disp(['Grad norm: ', num2str(norm(sol.grad))])
disp(['Hess norm: ', num2str(norm(sol.hessian))])

%%
load('reprocessedmarchlinedup')
trialplot=processeddata15;
start=25;
done=75;
tdelay=0;
tvec=trialplot(1,start:done);
uvec=[trialplot(32,start:done);trialplot(8,start:done)];
x0=trialplot([2,3,20],start);
p=sol.p;
[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);
 figure
subplot(2,1,1)
plot(simx(1,:),simx(2,:))
hold on
plot(trialplot(2,start:done),trialplot(3,start:done))
xlabel('X')
ylabel('Y')
legend('simulated','actual')

subplot(2,1,2)
plot(tvec,simx(3,:))
hold on
plot(trialplot(1,start:done),trialplot(20,start:done))
xlabel('time')
ylabel('heading')
legend('simulated','actual')

% subplot(5,1,3)
% plot(tvec,simx(4,:))
% hold on
% plot(tvec,trialplot(10,start:done))
% xlabel('time (s)')
% ylabel('lat v')
% legend('simulated','actual')
% 
% subplot(5,1,4)
% plot(tvec,simx(5,:))
% hold on
% plot(tvec,trialplot(26,start:done))
% xlabel('time (s)')
% ylabel('yaw rate')
% legend('simulated','actual')
% 
% 
% subplot(5,1,5)
% plot(tvec,uvec(1,:))
% hold on
% plot(tvec,uvec(2,:))
% xlabel('time (s)')
% ylabel('input')
% legend('steering','vx')
