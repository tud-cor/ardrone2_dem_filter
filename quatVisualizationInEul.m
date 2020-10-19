%% Initialization
clear;
close all;
clc;


%% Load quaternion data
% Retrieve bag file
cd ~/.ros;
bag = rosbag("angleTestMoveDroneManually2.bag");
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

% Select topics that need to be stored
% Controller signal topics
topics.cmdVel = 0;

% Simulation/flight data topics
topics.modelInput = 0;
topics.gazeboModelStates = 0;
topics.optitrack = 0;
topics.ardroneImu = 0;
topics.ardroneNav = 0;
topics.ardroneOdom = 1;
topics.rotorsMotorSpeed = 0;

% Set time interval with respect to start of rosbag recording
time = [17,41];


%% Get data
topicsOut = storeBagdata(bag,topics,time);


%% Set data
t = topicsOut.ardroneOdom.stampTime - topicsOut.ardroneOdom.stampTime(1);
orientQuat = topicsOut.ardroneOdom.orient;


%% Convert to Euler angles
orient = quat2EulAndWrap(orientQuat,0);


%% Plot
figure('Name','Orientation');
plot(t,orient(1,:));
hold on;
plot(t,orient(2,:));
plot(t,orient(3,:));
yline(0);
legend('phi','theta','psi','0 ref');
