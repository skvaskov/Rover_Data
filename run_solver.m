user.verbose=1;
tic
[sol,prob] = user.modelFit() ;
toc

disp(['Param fit: ', mat2str(sol.p)])
disp(['Grad norm: ', num2str(norm(sol.grad))])
disp(['Hess norm: ', num2str(norm(sol.hessian))])

%%
load('smoothdata100imutime.mat')
trialplot=processeddata17;
start=10;
done=150;
tvec=trialplot(1,start:done)-trialplot(1,start);
uvec=[steeringmodel(trialplot(32,start:done));trialplot(8,start:done)];
x0=trialplot([10,26],start);
p=sol.p;
[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);
subplot(2,1,1)
plot(tvec,simx(1,:))
hold on
plot(tvec,trialplot(10,start:done))
subplot(2,1,2)
plot(tvec,simx(2,:))
hold on
plot(tvec,trialplot(26,start:done))

% % subplot(5,1,1) 
% % plot(simx(1,:),simx(2,:))
% % hold on
% % plot(trialplot(2,:),trialplot(3,:))
% % legend('Simulated','Actual')
% % xlabel('X (m)')
% % ylabel('Y (m)')
% % % 
% % subplot(5,1,2)
% % plot(tvec,simx(3,:))
% % hold on
% % plot(trialplot(1,:),trialplot(20,:))
% % legend('Simulated','Actual')
% % xlabel('time')
% % ylabel('Heading(rad)')
% 
% subplot(3,1,1)
% plot(tvec,simx(1,:))
% hold on
% plot(trialplot(1,:),trialplot(10,:))
% legend('Simulated','Actual')
% xlabel('time')
% ylabel('vy')
% 
% 
% subplot(3,1,2)
% plot(tvec,simx(2,:))
% hold on
% plot(trialplot(1,:),trialplot(26,:))
% legend('Simulated','Actual')
% xlabel('Time (s)')
% ylabel('Yaw Rate (rad/s)')
% 
% 
% subplot(3,1,3)
% plot(tvec,uvec(1,:))
% hold on
% plot(tvec,uvec(2,:))
% xlabel('time(s)')
% ylabel('Input')
% legend('Steering','Velocity')

% 