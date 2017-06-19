user.verbose=1;
tic
[sol,prob] = user.modelFit() ;
toc

disp(['Param fit: ', mat2str(sol.p)])
disp(['Grad norm: ', num2str(norm(sol.grad))])
disp(['Hess norm: ', num2str(norm(sol.hessian))])

%%
[time,x,y,psi,vx,vy,w,throttle,steering] = mocap_data(dataslow,9,.2);

tvec=time';
uvec=throttle';
x0=vx(1);
p=sol.p;
[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);

figure
subplot(1,1,1)
plot(tvec,simx)
hold on
plot(tvec,vx)
xlabel('time (s)')
ylabel('distance')
legend('simulated','actual')

