%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot spiky force and torque data of tum_simulator
%
% Script to read out recorded force and torque data when simulating
% AR.Drone 2.0 using tum_simulator package and plot the resulting spiky
% data.
% 
% Author:        Dennis Benders, TU Delft, CoR
% Last modified: 21.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear;
close all;
clc;


%% Adjustable parameters
% Name of rosbag in ./ros
bagname = 'tum_simulator_spiky_data.bag';

% Topic selection
topics.cmdVel = 0;
topics.modelInput = 1;
topics.gazeboModelStates = 0;
topics.rotorsMotorSpeed = 0;
topics.optitrack = 0;
topics.ardroneImu = 0;
topics.ardroneNav = 0;
topics.ardroneOdom = 0;

time = [28,29];


%% Get bag data
% Retrieve bag file
defDir = pwd;
cd ~/.ros;
bag = rosbag(bagname);
cd(defDir);

topicsOut = storeBagdata(bag,topics,time);

if topics.modelInput
    modelInputStampTime = topicsOut.modelInput.stampTime;
    modelInputRecordTime = topicsOut.modelInput.recordTime;
    force = topicsOut.modelInput.force;
    torque = topicsOut.modelInput.torque;
end

modelInputStampTime = modelInputStampTime - modelInputStampTime(1);


%% Plot forces and torques
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;

lineStyles = {'-','--','-.',':'};
lineWidth = 3;

figure('Name','Forces');
hold on;
plot(modelInputStampTime,force(1,:));
plot(modelInputStampTime,force(2,:));
plot(modelInputStampTime,force(3,:));

figure('Name','Torques');
hold on;
plot(modelInputStampTime,torque(1,:));
plot(modelInputStampTime,torque(2,:));
plot(modelInputStampTime,torque(3,:));

figure('Name','\tau_x');
plot(modelInputStampTime,torque(1,:),'LineStyle',lineStyles{1},...
     'LineWidth',lineWidth);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('\tau_x','FontSize',labelFontSize);
title('Spiky data in torque around x-axis','FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
