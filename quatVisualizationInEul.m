%% Initialization
clear;
close all;
clc;


%% Load quaternion data
% Retrieve bag file
cd ~/.ros;
bag = rosbag("ardrone2_exp_2020-10-29_25_batP1.bag");
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

% Select topics that need to be stored
% Controller signal topics
topics.cmdVel = 0;

% Simulation/flight data topics
topics.modelInput = 0;
topics.gazeboModelStates = 0;
topics.optitrack = 1;
topics.ardroneImu = 0;
topics.ardroneNav = 0;
topics.ardroneOdom = 0;
topics.rotorsMotorSpeed = 0;

% Set time interval with respect to start of rosbag recording
time = [10,120];


%% Get data
topicsOut = storeBagdata(bag,topics,time);


%% Set data
t = topicsOut.optitrack.stampTime - topicsOut.optitrack.stampTime(1);

% Convert quaternion from ROS convention (x,y,z,w)
%                    to MATLAB convention (w,x,y,z)
orientQuat = [topicsOut.optitrack.orient(4,:);...
              topicsOut.optitrack.orient(1:3,:)];


%% Convert to Euler angles
orient = quat2eul(orientQuat','ZYX')';
% orient = quat2EulAndWrap(orientQuat,0);


%% Plot
figure('Name','Orientation in ZYX Euler angles');
plot(t,orient(3,:));
hold on;
plot(t,orient(2,:));
plot(t,orient(1,:));
yline(0);
legend('phi','theta','psi','0 ref','FontSize',20);
title('quat2eul','FontSize',30);
xlabel('Time (s)','FontSize',25);
ylabel('Angle (rad)','FontSize',25);


[~,idx] = min(abs(t-min(3,time(2))))
phiOffset = mean(orient(1,1:idx))
thetaOffset = mean(orient(2,1:idx))
psiOffset = mean(orient(3,1:idx))