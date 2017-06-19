% use linear regression to fit pwm model
% % a=(cm1/m-cm2/m*vx)(d)-cr/m-cd/m*vx^2
% clear all
% close all
% clc
% load('motor_pwm.mat')
% 
% for i=1:23
%     figure(1)
%     subplot(2,1,1)
%     plot(data(i).interp.mocap.longvelocity)
%     subplot(2,1,2)
%     plot(data(i).interp.input.throttle)
%     idxs(i)=input('Enter Start Index: ');
%     idxe(i)=input('Enter End Index: ');
% end
%%
clear all
close all
clc
%shift factorfor pwm

%build vecotrs of v,d,ax
% ax=[0;0];
% vx=[0;0];
% d=[(1580-sh)/500;(1430-sh)/500];
% A=[d -d.*vx -ones(size(d)) -vx.^2];
ax=[];
aximu=[];
vxenc=[];
vx=[];
Pi=[];
Pm=[];

%% select trials

threshold=.2;
load('motor_pwm.mat')
tnums=1:1:23;
tnums([1,5,8,9,15])=[];
%[Pm,vx,ax]=pwm_mocap_points(data,tnums,threshold,1);
[pa1,va1,aa1]=pwm_mocap_points(data1700,1:1:24,threshold,1);
 Pm=[Pm;pa1];
 vx=[vx;va1];
  ax=[ax;aa1];
 [pa2,va2,aa2]=pwm_mocap_points(dataslow,1:1:12,threshold,1);
Pm=[Pm;pa2];
vx=[vx;va2];
 ax=[ax;aa2];
%[Pi,vxenc,aximu]=pwm_imu_points(data,tnums,threshold,1);
 [pa3,va3,aa3]=pwm_imu_points(data1700,1:1:24,threshold,1);
 Pi=[Pi;pa3];
 vxenc=[vxenc;va3];
 aximu=[aximu;aa3];
[pa4,va4,aa4]=pwm_imu_points(dataslow,1:1:12,threshold,1);
Pi=[Pi;pa4];
vxenc=[vxenc;va4];
aximu=[aximu;aa4];
 
const=get_constv2(Pm,vx,ax);
const_imu=get_constv2(Pi,vxenc,aximu);


%% check answer 
% load('motor_pwm.mat')
% tnum=11;
% esta=est_axv2(data(tnum).interp.input.throttle,data(tnum).interp.mocap.longvelocity,const);
% esta_imu=est_axv2(data(tnum).interp.input.throttle,1.3/1.193*(data(tnum).interp.encoder.left+data(tnum).interp.encoder.right)/2,const_imu);
% figure(2)
% subplot(1,2,1)
% plot(data(tnum).interp.time,data(tnum).interp.mocap.longaccel_smooth,'k')
% hold on
% plot(data(tnum).interp.time,esta,'b--')
% plot(data(tnum).interp.time,esta_imu,'r--')
% legend('mocap', 'estimated','estimated w/ imu encoder')
% ylabel('acceleration')
% xlabel('time (s)')
% subplot(1,2,2)
% plot(data(tnum).interp.time,data(tnum).interp.input.throttle,'g-')
% ylim([1000,2000])
% ylabel('channel input')
% xlabel('time(s)')

tnum=11;
dt=dataslow;
pa=dt(tnum).interp.input.throttle;
va=dt(tnum).interp.mocap.longvelocity_smooth;
aa=dt(tnum).interp.imu.longaccel_smooth;
esta=est_axv2(pa,va,const);
vaimu=1.3/1.193*(dt(tnum).interp.encoder.left_smooth+dt(tnum).interp.encoder.right_smooth)/2;
esta_imu=est_axv2(pa,vaimu,const_imu);

figure(2)
subplot(1,2,1)
plot(dt(tnum).interp.time,aa,'k')
hold on
plot(dt(tnum).interp.time,esta,'b--')
plot(dt(tnum).interp.time,esta_imu,'r--')
legend('mocap', 'estimated','estimated w/ imu encoder')
ylabel('acceleration')
xlabel('time (s)')
subplot(1,2,2)
plot(dt(tnum).interp.time,pa,'g-')
ylim([1000,2000])
ylabel('channel input')
xlabel('time(s)')

% simulate

% %%
% ts=[1,4,.3,10];
% ys=[1660,1620,1400,1600];
% tssum=cumsum(ts);
% [T,Y]=ode45(@(t,y) motordyn(t,y,ys(1),const),[0,tssum(1)],0);
% for i=2:length(ts)
% [t,y]=ode45(@(t,y) motordyn(t,y,ys(i),const),[tssum(i-1),tssum(i)],Y(end));
% T=[T;t];
% Y=[Y;y];
% end
% figure(3)
% plot(T,Y)
% function dydt=motordyn(t,y,u,const)
%     dydt=[1 u y u*y y^2 u^2 y*u^2]*const;
% end
% 
