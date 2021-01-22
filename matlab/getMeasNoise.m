%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analyze measurement noise
%
% Script to calculate standard deviation of measurement noise and plot
% noise signal and its derivatives.
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
% topics.optitrack = 1;
% topics.ardroneImu = 0;
% topics.ardroneNav = 0;
% topics.ardroneOdom = 0;
% topics.rotorsMotorSpeed = 0;
% 
% % Set time interval with respect to start of rosbag recording
% time = [0,10];
% 
% 
% %% Get data
% topicsOut = storeBagdata(bag, topics, time);
% 
% if topics.optitrack
%     optitrackStampTime = topicsOut.optitrack.stampTime;
%     optitrackRecordTime = topicsOut.optitrack.recordTime;
%     optitrackPos = topicsOut.optitrack.pos;
%     optitrackOrientQuat = topicsOut.optitrack.orient;
% end
% 
% 
% %% Select suitable time frames
% % Plot data to search for time where quantities are constant
% optitrackPlotTime = optitrackStampTime - optitrackStampTime(1);
% figure('Name','OptiTrack position data');
% hold on;
% plot(optitrackPlotTime,optitrackPos(1,:),'-o');
% plot(optitrackPlotTime,optitrackPos(2,:),'-o');
% plot(optitrackPlotTime,optitrackPos(3,:),'-o');
% keyboard;
% 
% % Select data samples to use
% prompt      = {'Enter time of 1st data sample:',...
%                'Enter time of last data sample:'};
% dlgtitle    = 'Data selection';
% dims        = [1 35];
% definput    = {num2str(optitrackPlotTime(1)),...
%                num2str(optitrackPlotTime(end))};
% answer      = inputdlg(prompt,dlgtitle,dims,definput);
% startTime   = str2double(answer{1}) + optitrackStampTime(1);
% endTime     = str2double(answer{2}) + optitrackStampTime(1);
% [~,otStart] = min(abs(optitrackStampTime-startTime));
% [~,otEnd]   = min(abs(optitrackStampTime-endTime));
% 
% % Select proper OptiTrack data
% optitrackStampTime = optitrackStampTime(otStart:otEnd);
% optitrackStampTime = optitrackStampTime - optitrackStampTime(1);
% optitrackPos = optitrackPos(:,otStart:otEnd);
% optitrackOrientQuat = optitrackOrientQuat(:,otStart:otEnd);
% 
% 
% %% Ensure that all orientations are given in ZYX Euler angles, start at 0
% %  and are given in [rad]
% 
% % Convert quaternion from ROS convention (x,y,z,w)
% %                    to MATLAB convention (w,x,y,z)
% optitrackOrientQuat = [optitrackOrientQuat(4,:);...
%                        optitrackOrientQuat(1:3,:)];
% 
% % Convert quaternions to ZYX Euler angles: [Z;Y;X] ([psi;theta;phi])
% optitrackOrient = quat2eul(optitrackOrientQuat','ZYX')';
% 
% figure('Name','OptiTrack orientation data');
% subplot(3,1,1);
% plot(optitrackStampTime,optitrackOrient(1,:),'-o');
% title('\psi');
% subplot(3,1,2);
% plot(optitrackStampTime,optitrackOrient(2,:),'-o');
% title('\theta');
% subplot(3,1,3);
% plot(optitrackStampTime,optitrackOrient(3,:),'-o');
% title('\phi');
% 
% 
% %% Interpolate data
% % Sample time
% fs = 120;
% measNoiseData.sampleTime = 1/fs;
% 
% % OptiTrack data
% data.time = optitrackStampTime;
% data.value = optitrackOrient(3,:);
% tmp = interpolate(measNoiseData.sampleTime,data);
% time = tmp.time;
% z = tmp.value;
% 
% 
% %% Ensure data start at time 0 and data is zeroed
% time = time - time(1);
% z = z - z(1);
% 
% 
% %% Save data to speed up
% % OptiTrack data
% save('optitrackNoiseTest.mat','time','z');
% 
% 
%% Load data to speed up
% OptiTrack data
load optitrackNoiseTest.mat;


%% Calculate noise characteristics of OptiTrack roll angle states
std(z)
std(z)^2

[SigmaZEst1,sZEstGaussian] = estimateMeasNoiseCharacteristics(time,z,1,1);


%% Calculate higher-order derivatives of measurement noise
nZ = length(time);

zDer        = diff(z,1,2);
% zDer        = zDer - mean(zDer,2);
gausFitZDot = fitdist(zDer','Normal');

zDDer        = diff(zDer,1,2);
% zDDer        = zDDer - mean(zDDer,2);
gausFitZDDot = fitdist(zDDer','Normal');

zDer3        = diff(zDDer,1,2);
% zDer3        = zDer3 - mean(zDer3,2);
gausFitZD3   = fitdist(zDer3','Normal');

zDer4        = diff(zDer3,1,2);
% zDer4        = zDer4 - mean(zDer4,2);
gausFitZD4   = fitdist(zDer4','Normal');

zDer5        = diff(zDer4,1,2);
% zDer5        = zDer5 - mean(zDer5,2);
gausFitZD5   = fitdist(zDer5','Normal');

zDer6        = diff(zDer5,1,2);
% zDer6        = zDer6 - mean(zDer6,2);
gausFitZD6   = fitdist(zDer6','Normal');


%% Plot data
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;

% OptiTrack data
figure('Name','Measurement noise');
box on;
% subplot(2,1,1);
plot(time,z);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('\phi (rad)','FontSize',labelFontSize);
title('Measurement noise','FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
% subplot(2,1,2);
% plot(time,normrnd(0,std(z),[1,length(time)]));


axFontSize = 15;
labelFontSize = 20;
titleFontSize = 25;
figure('Name',['Gaussian distribution of measurement noise and '...
               'derivatives']);
box on;
% xLim = [-4e-4,4e-4];
subplot(3,1,1);
histfit(z,50,'normal');
% xlim(xLim);
legend('Histogram of measurement noise','Gaussian fit');
xlabel('Noise value (rad)','FontSize',labelFontSize);
ylabel('# occurences','FontSize',labelFontSize);
title('z','FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(3,1,2);
% histfit(zDer,57,'normal');
histfit(zDer,50,'normal');
% xlim(xLim);
legend('Histogram of 1st-order derivative','Gaussian fit');
xlabel('1st-order derivative noise value (rad/s)',...
       'FontSize',labelFontSize);
ylabel('# occurences','FontSize',labelFontSize);
title('1st-order derivative of z',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(3,1,3);
% histfit(zDDer,88,'normal');
histfit(zDDer,50,'normal');
% xlim(xLim);
legend('Histogram of 2nd-order derivative','Gaussian fit');
xlabel('2nd-order derivative noise value (rad/s^2)',...
       'FontSize',labelFontSize);
ylabel('# occurences','FontSize',labelFontSize);
title('2nd-order derivative of z',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;


axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;
figure('Name',['Gaussian distribution of derivatives 3 and 4']);
subplot(2,1,1);
box on;
histfit(zDer3,50,'normal');
legend('Histogram of 3rd-order derivative','Gaussian fit');
xlabel('3rd-order derivative noise value (rad/s^3)',...
       'FontSize',labelFontSize);
ylabel('# occurences','FontSize',labelFontSize);
title('3rd-order derivative of z',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(2,1,2);
box on;
histfit(zDer4,50,'normal');
legend('Histogram of 4th-order derivative','Gaussian fit');
xlabel('4th-order derivative noise value (rad/s^4)',...
       'FontSize',labelFontSize);
ylabel('# occurences','FontSize',labelFontSize);
title('4th-order derivative of z',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;


figure('Name',['Gaussian distribution of derivatives 5 and 6']);
subplot(2,1,1);
box on;
histfit(zDer5,50,'normal');
legend('Histogram of 5th-order derivative','Gaussian fit');
xlabel('5th-order derivative noise value (rad/s^5)',...
       'FontSize',labelFontSize);
ylabel('# occurences','FontSize',labelFontSize);
title('5th-order derivative of z',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(2,1,2);
box on;
histfit(zDer6,50,'normal');
legend('Histogram of 6th-order derivative','Gaussian fit');
xlabel('6th-order derivative noise value (rad/s^6)',...
       'FontSize',labelFontSize);
ylabel('# occurences','FontSize',labelFontSize);
title('6th-order derivative of z',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
