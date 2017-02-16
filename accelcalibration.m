
    %calibrate accelerometer data, to meet two objectives
    %(1) when standing still, values should be [0,0,1]
    %(2) when moving straight acceleration will only have component in X
    %direction
clear all
load('trial2.mat')
araw=[accX';accY';accZ'];


A=[9.826129556490821315e+00 1.105448087359244859e-02 -4.505705534461542511e-02;...
1.105448087359184837e-02 9.849205825189187635e+00 3.365730156842951931e-02;...
-4.505705534461620226e-02 3.365730156842847848e-02 9.888877421200394480e+00];
b=[6.484171140149968399e-03;7.233596302086491014e-02;-2.683383240809078529e-01];

 acal=A\(araw-b);
 stilldata=acal(:,1:140);
 xavg2=mean(stilldata(1,:));
 zavg2=mean(stilldata(3,:));
 theta=atan2(xavg2,zavg2);
 
 ROT2=[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
 acal2=ROT2'*acal;
 
 ROT2'*[xavg2;0;zavg2]
 
 yavg3=mean(stilldata(2,:));
 zavg3=mean(stilldata(3,:));
 
 phi=atan2(yavg3,zavg3);
 ROT3=[1 0 0;0 cos(phi) sin(phi);0 -sin(phi) cos(phi)];
 acal3=9.81/zavg3*ROT3'*acal2;
 
 ROT3'*[0;yavg3;zavg3]

 straightdata=acal3(:,160:340);
 
g=zeros(1,length(straightdata(1,:)));
for i=1:length(g)
    g(i)=atan(straightdata(2,i)/straightdata(1,i));
end



subplot(3,1,1)
plot(acal3(1,:))
hold on
plot(acal3(2,:))
title('Rotated about X and Y')

subplot(3,1,2)
plot(acal2(1,:))
hold on
plot(acal2(2,:))
title('Rotated about Y')

subplot(3,1,3)
hold on
plot(acal(1,:))
plot(acal(2,:))
title('original')








