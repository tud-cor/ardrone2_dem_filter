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
% topics.ardroneNav = 1;
% topics.ardroneOdom = 1;
% topics.rotorsMotorSpeed = 0;
% 
% % Set time interval with respect to start of rosbag recording
% % time = [0,75]; %exp_07-24_2
% time = [135,180]; %exp_07-24_3
% % time = [0,36]; %exp_07-24_8
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
% 
% if topics.ardroneNav
%     ardroneNavStampTime  = topicsOut.ardroneNav.stampTime;
%     ardroneNavVLin       = topicsOut.ardroneNav.vLin;
%     ardroneNavRot        = topicsOut.ardroneNav.rot;
% end
% 
% if topics.ardroneOdom
%     ardroneOdomStampTime  = topicsOut.ardroneOdom.stampTime;
%     ardroneOdomVLin       = topicsOut.ardroneOdom.vLin;
%     ardroneOdomOrientQuat = topicsOut.ardroneOdom.orient;
% end
% 
% % Align time and start at 0
% [~,idx] = min([optitrackStampTime(1);ardroneNavStampTime(1);...
%                ardroneOdomStampTime(1)]);
% if idx == 1
%     ardroneNavStampTime  = ardroneNavStampTime  - optitrackStampTime(1);
%     ardroneOdomStampTime = ardroneOdomStampTime - optitrackStampTime(1);
%     optitrackStampTime   = optitrackStampTime   - optitrackStampTime(1);
% elseif idx == 2
%     optitrackStampTime   = optitrackStampTime   - ardroneNavStampTime(1);
%     ardroneOdomStampTime = ardroneOdomStampTime - ardroneNavStampTime(1);
%     ardroneNavStampTime  = ardroneNavStampTime  - ardroneNavStampTime(1);
% elseif idx == 3
%     optitrackStampTime   = optitrackStampTime   - ardroneOdomStampTime(1);
%     ardroneNavStampTime  = ardroneNavStampTime  - ardroneOdomStampTime(1);
%     ardroneOdomStampTime = ardroneOdomStampTime - ardroneOdomStampTime(1);
% end
% 
% 
% %% Convert to Euler angles in radians
% optitrackOrient   = quat2EulAndWrap(optitrackOrientQuat,0);
% ardroneNavOrient  = ardroneNavRot/180*pi;
% ardroneOdomOrient = quat2EulAndWrap(ardroneOdomOrientQuat,0);
% 
% 
% %% Save data for code speedup
% save('checkARDrone2AnglesExp7_24_3.mat',...
%      'optitrackStampTime','optitrackPos','optitrackOrient',...
%      'ardroneNavStampTime','ardroneNavVLin','ardroneNavOrient',...
%      'ardroneOdomStampTime','ardroneOdomVLin','ardroneOdomOrient');
% 
% 
%% Load data for code speedup
load checkARDrone2AnglesExp7_24_3.mat;


%% Calculate initial yaw angle at beginning of experiment
tAvg = 3;

% Corrected OptiTrack yaw value w.r.t. OptiTrack (x,y,z) coordinate frame
% (for Exp7_24_3: -0.1742)
[~,otAvgEnd] = min(abs(optitrackStampTime-tAvg));
psiOffset    = mean(optitrackOrient(3,1:otAvgEnd));
psiOtOffset  = mean(optitrackOrient(3,1:otAvgEnd)) + pi/2;

% AR.Drone 2.0 yaw value w.r.t. drone orientation when the AR.Drone
% 2.0 initialized its sensors
[~,navAvgEnd] = min(abs(ardroneNavStampTime-tAvg));
psiNavOffset  = mean(ardroneNavOrient(3,1:navAvgEnd));

[~,odomAvgEnd] = min(abs(ardroneOdomStampTime-tAvg));
psiOdomOffset  = mean(ardroneOdomOrient(3,1:odomAvgEnd));


%% Correct OptiTrack angle data
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


%% Correct navdata angle data w.r.t. OptiTrack (x,y,z) coordinate frame
% Correct phi and theta axes (scaling) and yaw values (offset and scaling)
% ardroneNavOrientC = [1,0,0;0,1,0;0,0,1]*ardroneNavOrient + ...
%                     [0;0;psiOtOffset-psiArOffset];
ardroneNavOrientUC = rotz(psiOtOffset/pi*180)*ardroneNavOrient + ...
                    [0;0;psiOtOffset-psiNavOffset];
ardroneNavOrientC = rotz((psiOtOffset+psiOtOffset)/pi*180)*...
                    ardroneNavOrient + [0;0;psiOtOffset-psiNavOffset];

% Correct phi and theta values (offset)
ardroneNavOrientUC = ardroneNavOrientUC - ...
                   [mean(ardroneNavOrientUC(1:2,1:navAvgEnd),2);0];
ardroneNavOrientC = ardroneNavOrientC - ...
                   [mean(ardroneNavOrientC(1:2,1:navAvgEnd),2);0];


%% Correct IMU/odometry angle data
%  (IMU and odometry orientation are the same)
% Approximately (assuming a configuration with orientation psi = pi):
% phi_c = -phi_i
% theta_c = -theta_i
% psi_c = psi_i (from moment of hovering on)

% Convert to navdata convention
ardroneOdomOrientUC = [-1,0,0;0,-1,0;0,0,1]*ardroneOdomOrient;

% Do the same as for navdata:
% Correct phi and theta axes (scaling) and yaw values (offset and scaling)
% ardroneOdomOrientC = [1,0,0;0,1,0;0,0,1]*ardroneOdomOrientC + ...
%                     [0;0;psiOtOffset-psiOdomOffset];
ardroneOdomOrientUC = rotz(psiOtOffset/pi*180)*ardroneOdomOrientUC + ...
                    [0;0;psiOtOffset-psiOdomOffset];
ardroneOdomOrientC = rotz((psiOtOffset+psiOdomOffset)/pi*180)*...
                     ardroneOdomOrient + [0;0;psiOtOffset-psiOdomOffset];

% Correct phi and theta values (offset)
ardroneOdomOrientUC = ardroneOdomOrientUC - ...
                   [mean(ardroneOdomOrientC(1:2,1:odomAvgEnd),2);0];
ardroneOdomOrientC = ardroneOdomOrientC - ...
                   [mean(ardroneOdomOrientC(1:2,1:odomAvgEnd),2);0];


%% Plot for comparison navdata and OptiTrack
figure('Name',['Optitrack orientation and AR.Drone 2.0 navdata initial '...
       'and corrected orientation data']);
subplot(2,1,1);
plot(ardroneNavStampTime,ardroneNavOrientUC(1,:));
hold on;
plot(ardroneNavStampTime,ardroneNavOrientUC(2,:));
% plot(ardroneNavStampTime,ardroneNavOrientUC(3,:));
plot(optitrackStampTime,optitrackOrient(1,:));
plot(optitrackStampTime,optitrackOrient(2,:));
% plot(optitrackStampTime,optitrackOrient(3,:));
yline(0);
legend('\phi_{nav}','\theta_{nav}',...
       '\phi_{optitrack}','\theta_{optitrack}',...
       '0 ref');
title('Uncompensated x and y axes');

subplot(2,1,2);
plot(ardroneNavStampTime,ardroneNavOrientC(1,:));
hold on;
plot(ardroneNavStampTime,ardroneNavOrientC(2,:));
% plot(ardroneNavStampTime,ardroneNavOrientC(3,:));
plot(optitrackStampTime,optitrackOrientC(1,:));
plot(optitrackStampTime,optitrackOrientC(2,:));
% plot(optitrackStampTime,optitrackOrientC(3,:));
yline(0);
legend('\phi_{nav}','\theta_{nav}',...
       '\phi_{optitrack}','\theta_{optitrack}',...
       '0 ref');
title('Compensated x and y axes');


%% Plot for comparison IMU/odometry and OptiTrack
figure('Name',['Optitrack orientation and AR.Drone 2.0 IMU/odometry '...
       'initial and corrected orientation data']);
subplot(2,1,1);
plot(ardroneOdomStampTime,ardroneOdomOrientUC(1,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomOrientUC(2,:));
% plot(ardroneOdomStampTime,ardroneOdomOrientUC(3,:));
plot(optitrackStampTime,optitrackOrient(1,:));
plot(optitrackStampTime,optitrackOrient(2,:));
% plot(optitrackStampTime,optitrackOrient(3,:));
yline(0);
legend('\phi_{imu/odom}','\theta_{imu/odom}',...
       '\phi_{optitrack}','\theta_{optitrack}',...
       '0 ref');
title('Uncompensated x and y axes');

subplot(2,1,2);
plot(ardroneOdomStampTime,ardroneOdomOrientC(1,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomOrientC(2,:));
% plot(ardroneOdomStampTime,ardroneOdomOrientC(3,:));
plot(optitrackStampTime,optitrackOrientC(1,:));
plot(optitrackStampTime,optitrackOrientC(2,:));
% plot(optitrackStampTime,optitrackOrientC(3,:));
yline(0);
legend('\phi_{imu/odom}','\theta_{imu/odom}',...
       '\phi_{optitrack}','\theta_{optitrack}',...
       '0 ref');
title('Compensated x and y axes');


%% Plot for navdata
figure('Name','Optitrack position and AR.Drone 2.0 navdata orientation');
subplot(3,1,1);
plot(optitrackStampTime,optitrackPos(1,:));
hold on;
plot(optitrackStampTime,optitrackPos(2,:));
plot(optitrackStampTime,optitrackPos(3,:));
yline(0);
legend('x','y','z','0 ref');

% figure('Name','Original navdata orientation');
subplot(3,1,2);
plot(ardroneNavStampTime,ardroneNavOrient(1,:));
hold on;
plot(ardroneNavStampTime,ardroneNavOrient(2,:));
plot(ardroneNavStampTime,ardroneNavOrient(3,:));
yline(0);
legend('\phi_i','\theta_i','\psi_i','0 ref');

% figure('Name','Corrected navdata orientation');
subplot(3,1,3);
plot(ardroneNavStampTime,ardroneNavOrientC(1,:));
hold on;
plot(ardroneNavStampTime,ardroneNavOrientC(2,:));
plot(ardroneNavStampTime,ardroneNavOrientC(3,:));
yline(0);
legend('\phi_c','\theta_c','\psi_c','0 ref');


%% Plot for IMU/odometry
figure('Name',['Optitrack position and AR.Drone 2.0 IMU/odometry '...
               'orientation']);
subplot(3,1,1);
plot(optitrackStampTime,optitrackPos(1,:));
hold on;
plot(optitrackStampTime,optitrackPos(2,:));
plot(optitrackStampTime,optitrackPos(3,:));
yline(0);
legend('x','y','z','0 ref');

% figure('Name','Original IMU/odometry orientation');
subplot(3,1,2);
plot(ardroneOdomStampTime,ardroneOdomOrient(1,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomOrient(2,:));
plot(ardroneOdomStampTime,ardroneOdomOrient(3,:));
yline(0);
legend('\phi_i','\theta_i','\psi_i','0 ref');

% figure('Name','Corrected IMU/odometry orientation');
subplot(3,1,3);
plot(ardroneOdomStampTime,ardroneOdomOrientC(1,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomOrientC(2,:));
plot(ardroneOdomStampTime,ardroneOdomOrientC(3,:));
yline(0);
legend('\phi_c','\theta_c','\psi_c','0 ref');


%% Plot comparison of corrected navdata and IMU/odometry data
figure('Name','Orientation of corrected navdata and IMU/odometry data');
plot(ardroneNavStampTime,ardroneNavOrientC(1,:));
hold on;
plot(ardroneNavStampTime,ardroneNavOrientC(2,:));
plot(ardroneNavStampTime,ardroneNavOrientC(3,:));

plot(ardroneOdomStampTime,ardroneOdomOrientC(1,:));
plot(ardroneOdomStampTime,ardroneOdomOrientC(2,:));
plot(ardroneOdomStampTime,ardroneOdomOrientC(3,:));

yline(0);
legend('\phi_{nav}','\theta_{nav}','\psi_{nav}',...
       '\phi_{imu/odom}','\theta_{imu/odom}','\psi_{imu/odom}',...
       '0 ref');
