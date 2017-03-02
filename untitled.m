
subplot(3,2,1)
plot(dxdt0(1,1:65),'b')
hold on
plot(simdxdt0(1,:),'r')
ylabel('dx/dt')

subplot(3,2,2)
plot(dxdt0(2,1:65))
hold on
plot(simdxdt0(2,:))
ylabel('dy/dt')

subplot(3,2,3)
plot(dxdt0(3,1:65))
hold on
plot(simdxdt0(3,:))
ylabel('dpsi/dt')

subplot(3,2,4)
plot(dxdt0(4,1:65))
hold on
plot(simdxdt0(4,:))
ylabel('dvy/dt')

subplot(3,2,5)
plot(dxdt0(5,1:65))
hold on
plot(simdxdt0(5,:))
ylabel('dw/dt')