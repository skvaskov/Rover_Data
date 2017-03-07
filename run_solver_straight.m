user.verbose=1;
tic
[sol,prob] = user.modelFit() ;
toc

disp(['Param fit: ', mat2str(sol.p)])
disp(['Grad norm: ', num2str(norm(sol.grad))])
disp(['Hess norm: ', num2str(norm(sol.hessian))])

%%
load('smoothdata100imutime.mat')
trialplot=processeddata9;
start=1;
done=200;
tvec=trialplot(1,start:done)-trialplot(1,start);
uvec=trialplot(30,start:done);
x0=trialplot([4,6],start);
p=sol.p;
[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);

figure
subplot(3,1,1)
plot(tvec,simx(1,:))
hold on
plot(tvec,trialplot(4,start:done))
xlabel('time (s)')
ylabel('distance')
legend('simulated','actual')

subplot(3,1,2)
plot(tvec,simx(2,:))
hold on
plot(tvec,trialplot(6,start:done))
xlabel('time (s)')
ylabel('speed')
legend('simulated','actual')

subplot(3,1,3)
plot(tvec,uvec)
xlabel('time (s)')
ylabel('input')
