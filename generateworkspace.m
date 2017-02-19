clear all
%enter trial number
trial='25';

%load RC data
rcname=['shreyas' trial '_mavros_rc_in.csv'];
rcexcel=readtable(rcname);
secsrc=table2array(rcexcel(:,5));
nsecsrc=table2array(rcexcel(:,6));
channel=table2array(rcexcel(:,9));
i1=cell(length(secsrc),1);
i2=cell(length(secsrc),1);
for i=1:length(secsrc)
    row=char(channel(i));
    split=strsplit(row,{'[',', '});
    i1(i)=split(2);
    i2(i)=split(4);
end
Input1=str2num(char(i1));
Input2=str2num(char(i2));
clearvars rcexcel split row i2 i1 i channel rcname

%% 
% %load imu data
imuname=['shreyas' trial '_mavros_imu_data.csv'];
imuexcel=readtable(imuname);
accX=table2array(imuexcel(:,20));
accY=table2array(imuexcel(:,21));
accZ=table2array(imuexcel(:,22));
angvX=table2array(imuexcel(:,15));
angvY=table2array(imuexcel(:,16));
angvZ=table2array(imuexcel(:,17));
secsimu=table2array(imuexcel(:,5));
nsecsimu=table2array(imuexcel(:,6));
clearvars imuexcel imuname
%% 
% 
% %load mocap data
filename = ['/Users/seanvaskov/Documents/MATLAB/Rover Data/mocap_4_jan_17/shreyas' trial '.txt'];
delimiter = '\t';
startRow = 2;
formatSpec = '%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
fclose(fileID);
mocaptime = dataArray{:, 1};
x1 = dataArray{:, 2};
y1 = dataArray{:, 3};
z1 = dataArray{:, 4};
x2 = dataArray{:, 5};
y2 = dataArray{:, 6};
z2 = dataArray{:, 7};
x3 = dataArray{:, 8};
y3 = dataArray{:, 9};
z3 = dataArray{:, 10};
clearvars filename delimiter startRow formatSpec fileID dataArray ans;


%% 
%select imu and mocap start points (index before first motion) and save
%workspace, functions included to guess should be checked for a couple of
%trials
start=20;
avg=mean(accX(1:start));
imuguess=0;
for i=start:length(accX)
    diff=abs(accX(i)-avg);
    if diff>.1
        imuguess=i-1;
        break;
    end
    avg=mean(accX(1:i));
end
for i=1:length(x1)
    if abs(x1(i))>.1
        start=i+20;
        break;
    end
end
avg=mean(x1(start-20:start));
mocapguess=0;
for i=start:length(x1)
    diff=abs(x1(i)-avg);
    if diff>.001
        mocapguess=i-1;
        break;
    end
    avg=mean(x1(start-20:i));
end

imustart=imuguess;
mocapstart=mocapguess;
clearvars i imuguess diff avg mocapguess start
save(['trial' trial '-jan4.mat'])
clearvars trial

%% 

    
