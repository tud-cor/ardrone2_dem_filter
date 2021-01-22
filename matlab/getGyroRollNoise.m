%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analyze gyroscope roll rate noise
%
% Script to calculate standard deviation of gyroscope roll rate noise and
% plot noise signal.
% 
% Author:        Dennis Benders, TU Delft, CoR
% Last modified: 21.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear;
close all;
clc;


%% Set variables
% % Retrieve bag file
% defDir = pwd;
% cd ~/.ros;
% bag = rosbag("ardrone2_exp_2020-10-29_25_batP1.bag");
% cd(defDir);
% 
% % Select topics that need to be stored
% % Controller signal topics
% topics.cmdVel = 0;
% 
% % Simulation/flight data topics
% topics.modelInput = 0;
% topics.gazeboModelStates = 0;
% topics.optitrack = 0;
% topics.ardroneImu = 1;
% topics.ardroneNav = 0;
% topics.ardroneOdom = 0;
% topics.rotorsMotorSpeed = 0;
% 
% % Set time interval with respect to start of rosbag recording
% time = [0,130];
% 
% 
% %% Get data
% topicsOut = storeBagdata(bag, topics, time);
% 
% if topics.ardroneImu
%     ardroneImuStampTime  = topicsOut.ardroneImu.stampTime;
%     ardroneImuRecordTime  = topicsOut.ardroneImu.recordTime;
%     ardroneImuOrientQuat = topicsOut.ardroneImu.orient;
%     ardroneImuVAng       = topicsOut.ardroneImu.vAng;
% end
% 
% 
% %% Select suitable time frames
% % Plot data to search for time where quantities are constant
% ardroneImuPlotTime = ardroneImuStampTime - ardroneImuStampTime(1);
% figure('Name','AR.Drone 2.0 IMU angular velocity');
% hold on;
% plot(ardroneImuPlotTime,ardroneImuVAng(1,:),'-o');
% keyboard;
% 
% 
% % Select first set of data samples
% prompt       = {'Enter index of 1st data sample:',...
%                 'Enter index of last data sample:'};
% dlgtitle     = 'Data selection';
% dims         = [1 35];
% definput     = {num2str(ardroneImuPlotTime(1)),...
%                 num2str(ardroneImuPlotTime(end))};
% answer       = inputdlg(prompt,dlgtitle,dims,definput);
% startTime    = str2double(answer{1}) + ardroneImuStampTime(1);
% endTime      = str2double(answer{2}) + ardroneImuStampTime(1);
% [~,imuStart] = min(abs(ardroneImuStampTime-startTime));
% [~,imuEnd]   = min(abs(ardroneImuStampTime-endTime));
% 
% ardroneImuStampTime1 = ardroneImuStampTime(imuStart:imuEnd);
% ardroneImuVAng1      = ardroneImuVAng(:,imuStart:imuEnd);
% 
% 
% % Select second set of data samples
% prompt       = {'Enter index of 1st data sample:',...
%                 'Enter index of last data sample:'};
% dlgtitle     = 'Data selection';
% dims         = [1 35];
% definput     = {num2str(ardroneImuPlotTime(1)),...
%                 num2str(ardroneImuPlotTime(end))};
% answer       = inputdlg(prompt,dlgtitle,dims,definput);
% startTime    = str2double(answer{1}) + ardroneImuStampTime(1);
% endTime      = str2double(answer{2}) + ardroneImuStampTime(1);
% [~,imuStart] = min(abs(ardroneImuStampTime-startTime));
% [~,imuEnd]   = min(abs(ardroneImuStampTime-endTime));
% 
% ardroneImuStampTime2 = ardroneImuStampTime(imuStart:imuEnd);
% ardroneImuVAng2      = ardroneImuVAng(:,imuStart:imuEnd);
% 
% 
% %% Interpolate data
% % Sample time
% fs = 200;
% ts = 1/fs;
% 
% % First part of data
% data.time = ardroneImuStampTime1;
% data.value = ardroneImuVAng1(1,:);
% tmp = interpolate(ts,data);
% time1 = tmp.time;
% x1 = tmp.value;
% 
% % Second part of data
% data.time = ardroneImuStampTime2;
% data.value = ardroneImuVAng2(1,:);
% tmp = interpolate(ts,data);
% time2 = tmp.time;
% x2 = tmp.value;
% 
% 
% %% Ensure data start at time 0 and data is zeroed
% time1 = time1 - time1(1);
% x1 = x1 - mean(x1);
% std(x1)
% 
% time2 = time2 - time2(1);
% x2 = x2 - mean(x2);
% std(x2)
% 
% time = [time1,time2 - time2(1) + time1(end) + ts];
% x = [x1,x2];
% x = x - mean(x);
% std(x)
% 
% figure;
% plot(time1,x1);
% figure;
% plot(time2,x2);
% figure;
% plot(time,x);
% 
% 
% %% Save data to speed up
% save('ardroneImuNoiseTest.mat','time','x','time1','x1','time2','x2');
% 
% 
%% Load data to speed up
load ardroneImuNoiseTest.mat;


%% Calculate standard deviation characteristics of AR.Drone 2.0 gyro roll
%  rate
std(x)


%% Plot data
% Define font sizes
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;

% OptiTrack data
figure('Name','Gyro roll rate');
box on;
% subplot(2,1,1);
plot(time,x);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\dot{\phi}$ (rad/s)','FontSize',labelFontSize,...
                              'Interpreter','latex');
title('Gyro roll rate noise','FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
% subplot(2,1,2);
% plot(time,normrnd(0,std(z),[1,length(time)]));
