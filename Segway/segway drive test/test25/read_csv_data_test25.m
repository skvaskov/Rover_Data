close all
clear all
system_format = 1;% If using Linux, = 1 ; Windows, =2;
pathname = cd;
if (system_format == 1)
    pathname = strcat(pathname, '/');% If you are using Linux
else
    pathname = strcat(pathname, '\');% If you are using Windows
end
foldername = dir(pathname);

figure
hold on
 experiment_num = 1;
for i=4:length(foldername)
   
    subfolder_name = strcat(pathname, foldername(i).name);
    subfolder = dir(subfolder_name);
    if (system_format == 1)
        subfolder_name = strcat(subfolder_name, '/');% If you are using Linux
    else
        subfolder_name = strcat(subfolder_name, '\');% If you are using Windows
    end
    
    %% Read segway_cmd_vel.csv file
    cmd_vel_name = strcat(subfolder_name, '_slash_segway_slash_cmd_vel.csv');
    cmd_vel = csvread(cmd_vel_name,1,0);
    % Data format is: rosbagTimestamp,linear,x,y,z,angular,x,y,z
    cmd_time = cmd_vel(:,1)/10^9;
    cmd_lin = cmd_vel(:,3);
    cmd_ang = cmd_vel(:,9)*1.5;%since my angular velocity limit is 1.5rad/s
    start_col = find(cmd_lin,1);% Find the first non-zero point
    start_time = cmd_time(start_col);
    cmd_time = cmd_time - start_time;
    

    plot(cmd_time,cmd_lin,'b--');
    
    %% Read feedback_wheel_odometry
    feedback_wheel_odometry_name = strcat(subfolder_name, '_slash_segway_slash_feedback_slash_wheel_odometry.csv');
    delimiter = ',';
    startRow = 2;
    % Format string for each line of text:
    %   column1: double (%f)
    %	column2: text (%q)
    %   column3: double (%f)
    %	column4: text (%q)
    %   column5: double (%f)
    %	column6: double (%f)
    %   column7: text (%q)
    %	column8: text (%q)
    %   column9: text (%q)
    %	column10: text (%q)
    %   column11: text (%q)
    %	column12: double (%f)
    %   column13: double (%f)
    %	column14: double (%f)
    %   column15: text (%q)
    %	column16: double (%f)
    %   column17: double (%f)
    %	column18: double (%f)
    %   column19: double (%f)
    %	column20: text (%q)
    %   column21: text (%q)
    %	column22: text (%q)
    %   column23: text (%q)
    %	column24: double (%f)
    %   column25: double (%f)
    %	column26: double (%f)
    %   column27: text (%q)
    %	column28: double (%f)
    %   column29: double (%f)
    %	column30: double (%f)
    %   column31: text (%q)
    % For more information, see the TEXTSCAN documentation.
    formatSpec = '%f%q%f%q%f%f%q%q%q%q%q%f%f%f%q%f%f%f%f%q%q%q%q%f%f%f%q%f%f%f%q%[^\n\r]';

    % Open the text file.
    fileID = fopen(feedback_wheel_odometry_name,'r');
    % Read columns of data according to format string.
    % This call is based on the structure of the file used to generate this
    % code. If an error occurs for a different file, try regenerating the code
    % from the Import Tool.
    dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false);
    % Close the text file.
    fclose(fileID);
    % Allocate imported array to column variable names
    % rosbagTimestamp = dataArray{:, 1};
    % header = dataArray{:, 2};
    % seq = dataArray{:, 3};
    % stamp = dataArray{:, 4};
    % secs = dataArray{:, 5};
    % nsecs = dataArray{:, 6};
    % frame_id = dataArray{:, 7};
    % child_frame_id = dataArray{:, 8};
    % pose = dataArray{:, 9};
    % pose1 = dataArray{:, 10};
    % position = dataArray{:, 11};
    % x = dataArray{:, 12};
    % y = dataArray{:, 13};
    % z = dataArray{:, 14};
    % orientation = dataArray{:, 15};
    % x1 = dataArray{:, 16};
    % y1 = dataArray{:, 17};
    % z1 = dataArray{:, 18};
    % w = dataArray{:, 19};
    % covariance = dataArray{:, 20};
    % twist = dataArray{:, 21};
    % twist1 = dataArray{:, 22};
    % linear = dataArray{:, 23};
    % x2 = dataArray{:, 24};
    % y2 = dataArray{:, 25};
    % z2 = dataArray{:, 26};
    % angular = dataArray{:, 27};
    % x3 = dataArray{:, 28};
    % y3 = dataArray{:, 29};
    % z3 = dataArray{:, 30};
    % covariance1 = dataArray{:, 31};
 
    feedback_wheel_time = dataArray{:, 5} + dataArray{:, 6}/10^9;
    feedback_wheel_time = feedback_wheel_time - start_time;
    feedback_wheel_lin = dataArray{:, 24};
    feedback_wheel_ang = dataArray{:, 30};
    plot(feedback_wheel_time,feedback_wheel_lin,'r-');
    % Clear temporary variables
    clearvars filename delimiter startRow formatSpec fileID dataArray ans;

    %% Read Read odometry_local_filtered
    odom_local_name = strcat(subfolder_name, '_slash_segway_slash_odometry_slash_local_filtered.csv');
    % Initialize variables.
    delimiter = ',';
    startRow = 2;
    % Format string for each line of text:
    %   column1: double (%f)
    %	column2: text (%q)
    %   column3: double (%f)
    %	column4: text (%q)
    %   column5: double (%f)
    %	column6: double (%f)
    %   column7: text (%q)
    %	column8: text (%q)
    %   column9: text (%q)
    %	column10: text (%q)
    %   column11: text (%q)
    %	column12: double (%f)
    %   column13: double (%f)
    %	column14: double (%f)
    %   column15: text (%q)
    %	column16: double (%f)
    %   column17: double (%f)
    %	column18: double (%f)
    %   column19: double (%f)
    %	column20: text (%q)
    %   column21: text (%q)
    %	column22: text (%q)
    %   column23: text (%q)
    %	column24: double (%f)
    %   column25: double (%f)
    %	column26: double (%f)
    %   column27: text (%q)
    %	column28: double (%f)
    %   column29: double (%f)
    %	column30: double (%f)
    %   column31: text (%q)
    % For more information, see the TEXTSCAN documentation.
    formatSpec = '%f%q%f%q%f%f%q%q%q%q%q%f%f%f%q%f%f%f%f%q%q%q%q%f%f%f%q%f%f%f%q%[^\n\r]';

    % Open the text file.
    fileID = fopen(odom_local_name,'r');
    dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false);
    % Close the text file.
    
    fclose(fileID);
    % Allocate imported array to column variable names   
%     rosbagTimestamp = dataArray{:, 1};
%     header = dataArray{:, 2};
%     seq = dataArray{:, 3};
%     stamp = dataArray{:, 4};
%     secs = dataArray{:, 5};
%     nsecs = dataArray{:, 6};
%     frame_id = dataArray{:, 7};
%     child_frame_id = dataArray{:, 8};
%     pose = dataArray{:, 9};
%     pose1 = dataArray{:, 10};
%     position = dataArray{:, 11};
%     x = dataArray{:, 12};
%     y = dataArray{:, 13};
%     z = dataArray{:, 14};
%     orientation = dataArray{:, 15};
%     x1 = dataArray{:, 16};
%     y1 = dataArray{:, 17};
%     z1 = dataArray{:, 18};
%     w = dataArray{:, 19};
%     covariance = dataArray{:, 20};
%     twist = dataArray{:, 21};
%     twist1 = dataArray{:, 22};
%     linear = dataArray{:, 23};
%     x2 = dataArray{:, 24};
%     y2 = dataArray{:, 25};
%     z2 = dataArray{:, 26};
%     angular = dataArray{:, 27};
%     x3 = dataArray{:, 28};
%     y3 = dataArray{:, 29};
%     z3 = dataArray{:, 30};
%     covariance1 = dataArray{:, 31};
    
    odometry_local_time = dataArray{:, 5} + dataArray{:, 6}/10^9;
    odometry_local_time = odometry_local_time - start_time;
    odometry_local_lin = dataArray{:, 24};
    odometry_local_ang = dataArray{:, 30};
    plot(odometry_local_time,odometry_local_lin,'y-');
    
    % Clear temporary variables
    clearvars filename delimiter startRow formatSpec fileID dataArray ans;
    %% plot parameters
    experiment_num = experiment_num + 1;
    legend('command velocities','feedback wheel odometry','odometry local filtered','Location','best')
    axis([-0.5 5 -0.5 1.5])
    xlabel('Time (s)')
    ylabel('Linear velocity (m/s)')
    title({'Test 25, 0m/s -> -0.9m/s','Linear acceleration limit: 3m/s^2'})
end