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
start=1;
done=200;
tvec=trialplot(1,start:done)-trialplot(1,start);
uvec=[steeringmodel(trialplot(32,start:done)-40);trialplot(8,start:done)];
x0=trialplot([2,3,20],start);
p=sol.p;
[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);
subplot(2,1,1)
plot(simx(1,:),simx(2,:))
hold on
plot(trialplot(2,start:done),trialplot(3,start:done))

subplot(2,1,2)
plot(tvec,simx(3,:))
hold on
plot(tvec,trialplot(20,start:done))


