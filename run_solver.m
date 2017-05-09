user.verbose=1;
tic
[sol,prob] = user.modelFit() ;
toc

disp(['Param fit: ', mat2str(sol.p)])
disp(['Grad norm: ', num2str(norm(sol.grad))])
disp(['Hess norm: ', num2str(norm(sol.hessian))])

%%
load('reprocessedmarchlinedup')
trialplot=processeddata8;
clearvars processeddata*
start=14;
done=125;
tdelay=0;
tvec=trialplot(1,start:done);
% uvec=[steeringmodel(trialplot(32,start:done));trialplot(8,start:done)];
uvec=trialplot(30,start:done);
% x0=trialplot([2,3,20,10,26],start);
p=sol.p;

x0=[trialplot([4,8],start);0];

[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);
figure
subplot(3,1,1)
plot(tvec,simx(1,:))
hold on
plot(tvec,trialplot(4,start:done))
xlabel('time (s)')
ylabel('s (m)')
legend('simulated','data')
subplot(3,1,2)
plot(tvec,simx(2,:))
hold on
plot(tvec,trialplot(8,start:done))
xlabel('time (s)')
ylabel('vx (m/s)')
subplot(3,1,3)
plot(tvec,uvec)
% figure
% subplot(5,1,1)
% plot(simx(1,:),simx(2,:))
% hold on
% plot(trialplot(2,start:done),trialplot(3,start:done))
% xlabel('X')
% ylabel('Y')
% legend('simulated','actual')
% 
% subplot(5,1,2)
% plot(tvec,simx(3,:))
% hold on
% plot(trialplot(1,start:done),trialplot(20,start:done))
% xlabel('time')
% ylabel('heading')
% legend('simulated','actual')
% 
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
