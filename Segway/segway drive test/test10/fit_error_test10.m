%fit polynomials p(t) to data to determine error. Run
%read_segway_csv_test10.m must be perofmaed (line 8)


clear all
close all
 run read_segway_csv_test10.m

%fit odometry and feedback data from 0 to 0.5 m/s. Data is in form
%[time;data] and includes the command velocity once the 
data1=[odometry_local_time(113:181), odometry_local_lin(113:181);...
    feedback_wheel_time(354:598),feedback_wheel_lin(354:598)];
data1=sortrows(data1);
p1=polyfit(data1(:,1),data1(:,2),9);
t=linspace(data1(1,1),data1(end,1));
data1_fit=polyval(p1,t);
fig
hold on
plot(t,data1_fit,'k')

%fit odometry and feedback data from 0.5 m/s to 1 m/s. Data is in form
%[time;data] and includes the command velocity once the 
data2=[odometry_local_time(181:243), odometry_local_lin(181:243);...
    feedback_wheel_time(598:802),feedback_wheel_lin(598:802)];
data2=sortrows(data2);
p1=polyfit(data2(:,1),data2(:,2),9);
t=linspace(data2(1,1),data2(end,1));
data2_fit=polyval(p1,t);
fig
hold on
plot(t,data2_fit,'k')

%fit odometry and feedback data from 1 m/s to 0 m/s. Data is in form
%[time;data] and includes the command velocity once the 
data3=[odometry_local_time(243:304), odometry_local_lin(243:304);...
    feedback_wheel_time(802:1003),feedback_wheel_lin(802:1003)];
data3=sortrows(data3);
p1=polyfit(data3(:,1),data3(:,2),7);
t=linspace(data3(1,1),data3(end,1));
data3_fit=polyval(p1,t);
fig
hold on
plot(t,data3_fit,'k')


