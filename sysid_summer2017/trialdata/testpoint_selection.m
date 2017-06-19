clear all
close all
clc

P_i=[1,-1,-1,1;1,1,-1,-1;0,0,0,0];
yaw=[pi/6,pi/6,pi/6];
pitch=[0,-pi/10,pi/10];
roll=[0,0,pi/5];
Rot=zeros(3,3,length(yaw));
Rot_global=zeros(3,3,length(yaw));
P=zeros(3,4,length(yaw));
for i=1:length(yaw)
    OBA=[cos(yaw(i)),sin(yaw(i)),0;-sin(yaw(i)),cos(yaw(i)),0;0,0,1];
    OCB=[cos(pitch(i)),0,-sin(pitch(i));0,1,0;sin(pitch(i)),0,cos(pitch(i))];
    ODC=[1,0,0;0,cos(roll(i)),sin(roll(i));0,-sin(roll(i)),cos(roll(i))];
    Rot(:,:,i)=(ODC*OCB*OBA)';
    if i==1
        Rot_global(:,:,1)=(ODC*OCB*OBA)';
        P(:,:,i)=Rot(:,:,i)*P_i;
    else
        Rot_global(:,:,i)=Rot(:,:,i)*Rot_global(:,:,i-1);
        P(:,:,i)=Rot(:,:,i)*P(:,:,i-1);
    end
end
    

[U(1),~,~]=absor(P_i,P(:,:,1));
[U(2),~,~]=absor(P(:,:,1),P(:,:,2));
[U(3),~,~]=absor(P(:,:,2),P(:,:,3));
for i=1:3
yaw_step(i)=atan2(U(i).R(2,1),U(i).R(1,1));
pitch_step(i)=atan2(-U(i).R(3,1),sqrt(U(i).R(3,2)^2+U(i).R(3,3)^2));
roll_step(i)=atan2(U(i).R(3,2),U(i).R(3,3));
end
U_global=U(3).R*U(2).R*U(1).R
yaw_angle=atan2(U_global(2,1),U_global(1,1));
pitch_angle=atan2(-U_global(3,1),sqrt(U_global(3,2)^2+U_global(3,3)^2));
roll_angle=atan2(U_global(3,2),U_global(3,3));
P_end=U_global*P_i;