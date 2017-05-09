%Run this script when you are in the folder that has all the subfolders (test25_i...) you
%want to get data from in it. Note the spreadsheet names for the odometry
%data and fedback data are different than the old tests
%% setup
clear ; clc  ; close all
pathname = cd;
pathname = strcat(pathname, '/');
tstart=0; %leave as zero unless for some reason you want to fit around the command
ad=1; %set to 0 if command is accelerating, 1 if decellerating
%***The code finds the acceleration and deceleration command and sets that
%to t=0 (independent of tstart. tsart is only used for the polyonmial fitting)
tend = tstart + 1 ;
%% parameters
test_num = '25' ;

gorder = 6 ;

foldername = dir(pathname);

en=1;
trialdata=struct('cmd_vel',{},'cmd_time',{},'cmd_lin',{},...
    'feedback_wheel_time',{},'feedback_wheel_lin',{},'feedback_wheel_ang',{},...
    'odometry_local_time',{},'odometry_local_lin',{},'odometry_local_ang',{});
for i=5:length(foldername)
    %get name of foler for each test
    subfolder_name = strcat(pathname, foldername(i).name);
    subfolder = dir(subfolder_name);
    subfolder_name = strcat(subfolder_name, '/');
     
cmd_vel = csvread([subfolder_name,'/_slash_segway_slash_cmd_vel.csv'],1,0);


% Data format is: rosbagTimestamp,linear,x,y,z,angular,x,y,z
cmd_time = cmd_vel(:,1)/10^9;
cmd_lin = cmd_vel(:,3);
if ad==0
start_col = find(cmd_lin,1);% Find the first non-zero point
end
if ad==1 % Find the last non-zero point
idx=find(cmd_lin);
start_col=idx(end);
end
start_time = cmd_time(start_col);
cmd_time = cmd_time - start_time;

%addd cmd to structure and clear temporary variables
trialdata(en).cmd_vel=cmd_vel;
trialdata(en).cmd_time=cmd_time;
trialdata(en).cmd_lin=cmd_lin;
clearvars cmd_lin cmd_time cmd_vel

%read feedback wheel odometery (copied from Fan's read_csv code, refer to
%that for comments)
feedback_wheel_odometry_name = strcat(subfolder_name, '_slash_segway_slash_feedback_slash_wheel_odometry.csv');
 delimiter = ',';
 startRow = 2;
 formatSpec = '%f%q%f%q%f%f%q%q%q%q%q%f%f%f%q%f%f%f%f%q%q%q%q%f%f%f%q%f%f%f%q%[^\n\r]';
 fileID = fopen(feedback_wheel_odometry_name,'r');
  dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false);
  fclose(fileID);
  
feedback_wheel_time = dataArray{:, 5} + dataArray{:, 6}/10^9;
feedback_wheel_time = feedback_wheel_time - start_time;
feedback_wheel_lin = dataArray{:, 24};
feedback_wheel_ang = dataArray{:, 30};
clearvars filename delimiter startRow formatSpec fileID dataArray ans;

%addd feedback to structure and clear temporary variables
trialdata(en).feedback_wheel_time=feedback_wheel_time;
trialdata(en).feedback_wheel_lin=feedback_wheel_lin;
trialdata(en).feedback_wheel_ang=feedback_wheel_ang;
clearvars feedback_wheel_time feedback_wheel_lin feedback_wheel_ang feedback_wheel_odometry_name

%read feedback wheel odometery (copied from Fan's read_csv code, refer to
%that for comments)
odom_local_name = strcat(subfolder_name, '_slash_segway_slash_odometry_slash_local_filtered.csv');
delimiter = ',';
startRow = 2;
formatSpec = '%f%q%f%q%f%f%q%q%q%q%q%f%f%f%q%f%f%f%f%q%q%q%q%f%f%f%q%f%f%f%q%[^\n\r]';
fileID = fopen(odom_local_name,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false);
fclose(fileID);

odometry_local_time = dataArray{:, 5} + dataArray{:, 6}/10^9;
odometry_local_time = odometry_local_time - start_time;
odometry_local_lin = dataArray{:, 24};
odometry_local_ang = dataArray{:, 30};
clearvars filename delimiter startRow formatSpec fileID dataArray ans;

%add odometry to structure and clear temporary variables
trialdata(en).odometry_local_time=odometry_local_time;
trialdata(en).odometry_local_lin=odometry_local_lin;
trialdata(en).odometry_local_ang=odometry_local_ang;
clearvars odometry_local_time odometry_local_lin odometry_local_ang odom_local_name start_time start_col

en=en+1;
end

%clear variables used for data extraction
clearvars subfolder subfolder_name test foldername pathname

%combine and sort data clipped from tsart to tend  into one array for fitting
en=en-1;
odomdata=[];
feedbackdata=[];
cmddata=[];
for i=1:en
    ctempV=trialdata(i).cmd_lin;
    ctempT=trialdata(i).cmd_time;
    cmddata=[cmddata;ctempT(ctempT>=tstart & ctempT<=tend),ctempV(ctempT>=tstart & ctempT<=tend)];
    otempV=trialdata(i).odometry_local_lin;
    otempT=trialdata(i).odometry_local_time;
    odomdata=[odomdata;otempT(otempT>=tstart & otempT<=tend),otempV(otempT>=tstart & otempT<=tend)];
    ftempV=trialdata(i).feedback_wheel_lin;
    ftempT=trialdata(i).feedback_wheel_time;
    feedbackdata=[feedbackdata;ftempT(ftempT>=tstart & ftempT<=tend),ftempV(ftempT>=tstart & ftempT<=tend)];
end
clearvars otempV otempT ftempV ftempT ctempV ctempT i
cmddata=sortrows(cmddata);
odomdata=sortrows(odomdata);
feedbackdata=sortrows(feedbackdata);
alldata=sortrows([odomdata;feedbackdata]);
% plot snippet of alldata (time=time-tsart)
figure(1) ; hold on
plot(cmddata(:,1)-tstart,cmddata(:,2))
plot(feedbackdata(:,1)-tstart,feedbackdata(:,2),'r.')
plot(odomdata(:,1)-tstart,odomdata(:,2),'m.')
%% polyfit
gcoeffOdom = polyfit(odomdata(:,1) - tstart,odomdata(:,2),gorder) ;
tvals = linspace(0,1,300) ;
gvalsOdom = polyval(gcoeffOdom,tvals) ;
plot(tvals,gvalsOdom,'g','LineWidth',2)

gcoeffFback = polyfit(feedbackdata(:,1) - tstart,feedbackdata(:,2),gorder) ;
gvalsFback = polyval(gcoeffFback,tvals) ;
plot(tvals,gvalsFback,'y','LineWidth',2)

% finalize plop
plot([0,1],[1 1],'r--')
legend('command velocities','odometry','feedback','g (odometry)','g (feedback)','Location','best')
xlabel('Time (s)')
ylabel('Linear velocity (m/s)')
title('Test 25, 0m/s -> 1m/s, 3m/s^2')

% create symbolic poly
syms t real
pOdom = poly2sym(gcoeffOdom,t);
pFback=poly2sym(gcoeffFback,t);