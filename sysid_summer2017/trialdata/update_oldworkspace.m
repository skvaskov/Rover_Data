
clear all
close all
clc
%enter trial name
trialname='circles3_5_31_workspace.mat';
%enter file path to place where raw data is stored
filepath='~/Documents/MATLAB/Rover_Data/sysid_summer2017/trialdata/workspaces/';
load([filepath,trialname])

orientationimu(3,:)=atan2(2*(orientW.*orientZ+orientX.*orientY),1-2*(orientY.^2+orientZ.^2));
orientationimu(2,:)=asin(2*(orientW.*orientY-orientZ.*orientX));
orientationimu(1,:)=atan2(2*(orientW.*orientX+orientY.*orientZ),1-2*(orientX.^2+orientY.^2));

load('imucal_circles.mat')
orientationimu_cal=regParams.R*unwrap(orientationimu,[],2)+regParams.t;

clearvars Z Y X CPos
