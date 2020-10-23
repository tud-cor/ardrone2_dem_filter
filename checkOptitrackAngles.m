%% Initialization
clear;
close all;
clc;


%% Load quaternion data
% % Retrieve bag file
% cd ~/.ros;
% bag = rosbag("ardrone2_exp_2020-07-24_3.bag");
% cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;
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
% % time = [0,75]; %exp 07-24_2
% time = [135,180]; %exp 07-24_3
% % time = [0,36]; %exp 07-24_8
% 
% 
% %% Get data
% topicsOut = storeBagdata(bag,topics,time);
% 
% if topics.optitrack
%     optitrackStampTime  = topicsOut.optitrack.stampTime;
%     optitrackPos        = topicsOut.optitrack.pos;
%     optitrackOrientQuat = topicsOut.optitrack.orient;
% end
% optitrackStampTime = optitrackStampTime - optitrackStampTime(1);
% 
% 
% %% Convert to Euler angles
% optitrackOrient = quat2EulAndWrap(optitrackOrientQuat,0);
% 
% 
% %% Save data for code speedup
% save('checkOptitrackAnglesExp7_24_3.mat',...
%      'optitrackStampTime','optitrackPos','optitrackOrient');
% 
% 
%% Load data for code speedup
load checkOptitrackAnglesExp7_24_3.mat;


%% Calculate initial yaw angle at beginning of experiment w.r.t. OptiTrack
%  configuration
tAvg = 3;
[~,otAvgEnd] = min(abs(optitrackStampTime-tAvg));
psiOffset = mean(optitrackOrient(3,1:otAvgEnd));


%% Correct angle data
% Approximately (assuming a configuration with orientation psi = pi/2):
% phi_c = -theta_i
% theta_c = phi_i
% psi_c psi_i + pi/2
% optitrackOrientC = [0,-1,0;1,0,0;0,0,1]*optitrackOrient + [0;0;pi/2];

% Correct phi and theta axes (scaling) and yaw values (offset and scaling)
optitrackOrientC = rotz(-psiOffset/pi*180)*optitrackOrient + ...
                   [0;0;pi/2];

% Correct phi and theta values (offset)
optitrackOrientC = optitrackOrientC - ...
                   [mean(optitrackOrientC(1:2,1:otAvgEnd),2);0];


%% Plot
figure('Name','Position and orientation');
subplot(3,1,1);
plot(optitrackStampTime,optitrackPos(1,:));
hold on;
plot(optitrackStampTime,optitrackPos(2,:));
plot(optitrackStampTime,optitrackPos(3,:));
yline(0);
legend('x','y','z','0 ref');

% figure('Name','Original orientation');
subplot(3,1,2);
plot(optitrackStampTime,optitrackOrient(1,:));
hold on;
plot(optitrackStampTime,optitrackOrient(2,:));
plot(optitrackStampTime,optitrackOrient(3,:));
yline(0);
legend('\phi_i','\theta_i','\psi_i','0 ref');

% figure('Name','Corrected orientation');
subplot(3,1,3);
plot(optitrackStampTime,optitrackOrientC(1,:));
hold on;
plot(optitrackStampTime,optitrackOrientC(2,:));
plot(optitrackStampTime,optitrackOrientC(3,:));
yline(0);
legend('\phi_c','\theta_c','\psi_c','0 ref');
