%% Initialization
clear;
close all;
clc;


%% Load quaternion data
% Retrieve bag file
cd ~/.ros;
bag = rosbag("angleTestR90PCombined.bag");
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

% Select topics that need to be stored
% Controller signal topics
topics.cmdVel = 0;

% Simulation/flight data topics
topics.modelInput = 0;
topics.gazeboModelStates = 0;
topics.optitrack = 0;
topics.ardroneImu = 1;
topics.ardroneNav = 1;
topics.ardroneOdom = 1;
topics.rotorsMotorSpeed = 0;

% Set time interval with respect to start of rosbag recording
time = [0,18];


%% Get data
topicsOut = storeBagdata(bag,topics,time);


%% Set data
imuTime = topicsOut.ardroneImu.stampTime - ...
          topicsOut.ardroneImu.stampTime(1);
imuOrientQuat = topicsOut.ardroneImu.orient;

navTime = topicsOut.ardroneNav.stampTime - ...
          topicsOut.ardroneNav.stampTime(1);
navOrient = topicsOut.ardroneNav.rot;

odomTime = topicsOut.ardroneOdom.stampTime - ...
           topicsOut.ardroneOdom.stampTime(1);
odomOrientQuat = topicsOut.ardroneOdom.orient;


%% Convert to Euler angles
imuOrient = quat2EulAndWrap(imuOrientQuat,0);
navOrient = navOrient/180*pi;
odomOrient = quat2EulAndWrap(odomOrientQuat,0);


%% Get error after movement using navdata
% switchNeeded = 1;
% if switchNeeded
%     tSwitch = 35;
%     [~,sSwitch] = min(abs(navTime-tSwitch));
% 
%     phiErrIH   = navOrient(1,sSwitch) - navOrient(1,1);
%     thetaErrIH = navOrient(2,sSwitch) - navOrient(2,1);
%     psiErrIH   = navOrient(3,sSwitch) - navOrient(3,1);
% 
%     phiErrRPY   = navOrient(1,end) - navOrient(1,sSwitch);
%     thetaErrRPY = navOrient(2,end) - navOrient(2,sSwitch);
%     psiErrRPY   = navOrient(3,end) - navOrient(3,sSwitch);
% else
%     phiErrIH   = navOrient(1,end) - navOrient(1,1);
%     thetaErrIH = navOrient(2,end) - navOrient(2,1);
%     psiErrIH   = navOrient(3,end) - navOrient(3,1);
% end


%% Plot
figure('Name','Orientation IMU in ZYX Euler angles');
subplot(3,1,1);
plot(imuTime,imuOrient(1,:));
hold on;
plot(imuTime,imuOrient(2,:));
plot(imuTime,imuOrient(3,:));
yline(0);
legend('phi','theta','psi','0 ref','FontSize',20);
title('quat2eul IMU','FontSize',30);
xlabel('Time (s)','FontSize',25);
ylabel('Angle (rad)','FontSize',25);

subplot(3,1,2);
plot(navTime,navOrient(1,:));
hold on;
plot(navTime,navOrient(2,:));
plot(navTime,navOrient(3,:));
yline(0);
legend('phi','theta','psi','0 ref','FontSize',20);
title('quat2eul navdata','FontSize',30);
xlabel('Time (s)','FontSize',25);
ylabel('Angle (rad)','FontSize',25);

subplot(3,1,3);
plot(odomTime,odomOrient(1,:));
hold on;
plot(odomTime,odomOrient(2,:));
plot(odomTime,odomOrient(3,:));
yline(0);
legend('phi','theta','psi','0 ref','FontSize',20);
title('quat2eul odometry','FontSize',30);
xlabel('Time (s)','FontSize',25);
ylabel('Angle (rad)','FontSize',25);
