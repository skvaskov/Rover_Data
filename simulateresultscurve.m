%simulate results for random inputs
datastruct = load('smoothdata100imutime.mat') ;
tarray=[22,17,10];
carray=['r','b','g','c','m'];

Izg=.5;
cag=.5;

l=.29;
lf=.18;
m=2.759;
w=.3;
Izub=2.759/12*(l^2+w^2);
%Izg=2.759/12*l^2;
lflb=.1585;
%p=[m,Izg,1.8,2.9,cag,1]';
%p= [3.132187972709897; 0.997594187931241; 0.686470999900527; 2.562831941269615; 1.411292653719778;  0.802005305120440];
%p= [2.759; 0.997594187931241; 0.686470999900527; 2.562831941269615;  1.243142641992868;  0.802005305120440];
p=[2.75910149211392;1.70118417936534;1.58507316334463;2.99992456715352;1.4076866985652;1.19863040386614];
%p=[2.75910149211392;1.70118417936534;2.5;2.99992456715352;1;1;1;1;1;1] ;
start=1;
fin=150;

figure
for i=1:length(tarray)
trial= datastruct.(['processeddata' num2str(tarray(i))]) ;

tvec=trial(1,start:fin)-trial(1,start);

uvec=[steeringmodel(trial(32,start:fin));trial(8,start:fin)];

fdyn=@lygerostan;

x0=trial([2 3 20 10 26],start);

[simx, simdxdt, simdfdx, simdfdp]=simulateDynamicsWithInput(fdyn,tvec,uvec,x0,p);


subplot(4,1,1)
plot(simx(1,:),simx(2,:),[carray(i),'--'])
hold on
plot(trial(2,start:fin),trial(3,start:fin),carray(i))

subplot(4,1,2)
plot(tvec,simx(3,:),[carray(i),'--'])
hold on
plot(tvec,trial(20,start:fin),carray(i))

subplot(4,1,3)
plot(tvec,simx(4,:),[carray(i),'--'])
hold on
plot(tvec,trial(10,start:fin),carray(i))

subplot(4,1,4)
plot(tvec,simx(5,:),[carray(i),'--'])
hold on
plot(tvec,trial(26,start:fin),carray(i))
end

subplot(4,1,1)
xlabel('X')
ylabel('Y')
legend('simulated','actual')

subplot(4,1,2)
xlabel('time (s)')
ylabel('heading')
legend('simulated','actual')

subplot(4,1,3)
xlabel('time (s)')
ylabel('lat v')
legend('simulated','actual')

subplot(4,1,4)
xlabel('time (s)')
ylabel('yaw rate')
legend('simulated','actual')