%% Initialization
clear;
close all;
clc;


%% Data to compare
% Possible state    | Available on topic
% --------------------------------------
% x                 |                   ardrone/odometry (drifting) TODO check with coordinate transforms
% y                 |                   ardrone/odometry (drifting) TODO check with coordinate transforms
% z                 | ardrone/navdata + ardrone/odometry (same, but a little bit of offset!) TODO check with coordinate transforms

% xDot              | ardrone/navdata + ardrone/odometry (same) TODO Check with Optitrack data for correctnes
% yDot              | ardrone/navdata + ardrone/odometry (same) TODO Check with Optitrack data for correctnes
% zDot              | ardrone/navdata + ardrone/odometry (same, but 0!)

% phi               | ardrone/imu + ardrone/navdata + ardrone/odometry     TODO check with coordinate transform + initialization
% theta             | ardrone/imu + ardrone/navdata + ardrone/odometry     TODO check with coordinate transform + initialization
% psi               | ardrone/imu + ardrone/navdata + ardrone/odometry(2x) TODO check with coordinate transform + initialization

% phiDot            | ardrone/imu + ardrone/odometry (odom is 0!) TODO check with coordinate transform
% thetaDot          | ardrone/imu + ardrone/odometry (odom is 0!) TODO check with coordinate transform
% psiDot            | ardrone/imu + ardrone/odometry (odom is 0!) TODO check with coordinate transform


%% Data to be used to run the filters
% Possible state    | Chosen topic
% --------------------------------
% x                 | 
% y                 | 
% z                 | 

% xDot              | 
% yDot              | 
% zDot              | 

% phi               | 
% theta             | 
% psi               | 

% phiDot            | 
% thetaDot          | 
% psiDot            | 


%% Set variables
% % Retrieve bag file
% cd ~/.ros;
% bag = rosbag("ardrone2_exp_2020-07-24_8.bag");
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
% topics.ardroneImu = 1;
% topics.ardroneNav = 1;
% topics.ardroneOdom = 1;
% topics.rotorsMotorSpeed = 0;
% 
% % Set time interval with respect to start of rosbag recording
% % time = [18,25];
% time = [0,20];
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
% if topics.ardroneImu
%     ardroneImuStampTime  = topicsOut.ardroneImu.stampTime;
%     ardroneImuOrientQuat = topicsOut.ardroneImu.orient;
%     ardroneImuVAng       = topicsOut.ardroneImu.vAng;
% end
% 
% if topics.ardroneNav
%     ardroneNavStampTime  = topicsOut.ardroneNav.stampTime;
%     ardroneNavAltd       = topicsOut.ardroneNav.altd;
%     ardroneNavVLin       = topicsOut.ardroneNav.vLin;
%     ardroneNavRot        = topicsOut.ardroneNav.rot;
% end
% 
% if topics.ardroneOdom
%     ardroneOdomStampTime  = topicsOut.ardroneOdom.stampTime;
%     ardroneOdomPos        = topicsOut.ardroneOdom.pos;
%     ardroneOdomVLin       = topicsOut.ardroneOdom.vLin;
%     ardroneOdomOrientQuat = topicsOut.ardroneOdom.orient;
%     ardroneOdomVAng       = topicsOut.ardroneOdom.vAng;
% end
% 
% 
% %% Save data to .mat for code speedup
% save('compARDrone2OptiTrackData.mat',...
%      'optitrackStampTime','optitrackPos','optitrackOrientQuat',...
%      'ardroneImuStampTime','ardroneImuOrientQuat','ardroneImuVAng',...
%      'ardroneNavStampTime','ardroneNavAltd',...
%      'ardroneNavVLin','ardroneNavRot',...
%      'ardroneOdomStampTime','ardroneOdomPos','ardroneOdomVLin',...
%      'ardroneOdomOrientQuat','ardroneOdomVAng');
% 
% 
%% Select data from .mat for code speedup
load compARDrone2OptiTrackData.mat;


%% Select suitable time frame
% Plot position data to determine the time frame visually
figure('Name','OptiTrack position data');
hold on;
plot(optitrackStampTime,optitrackPos(1,:),'-o');
plot(optitrackStampTime,optitrackPos(2,:),'-o');
plot(optitrackStampTime,optitrackPos(3,:),'-o');

% Select data samples to use
prompt   = {'Enter index of 1st data sample:',...
            'Enter index of last data sample:'};
dlgtitle = 'Data selection';
dims     = [1 35];
definput = {'1',num2str(length(optitrackStampTime))};
answer   = inputdlg(prompt,dlgtitle,dims,definput);
otStart  = round(str2double(answer{1}));
otEnd    = round(str2double(answer{2}));

% Select corresponding time frame in OptiTrack data
optitrackStampTime  = optitrackStampTime(otStart:otEnd);
optitrackPos        = optitrackPos(:,otStart:otEnd);
optitrackOrientQuat = optitrackOrientQuat(:,otStart:otEnd);

% Select corresponding time frame in ardroneImu data
[~,arStart] = min(abs(ardroneImuStampTime-optitrackStampTime(1)));
[~,arEnd]   = min(abs(ardroneImuStampTime-optitrackStampTime(end)));
ardroneImuStampTime  = ardroneImuStampTime(arStart:arEnd);
ardroneImuOrientQuat = ardroneImuOrientQuat(:,arStart:arEnd);
ardroneImuVAng       = ardroneImuVAng(:,arStart:arEnd);

% Select corresponding time frame in ardroneNav data
[~,arStart] = min(abs(ardroneNavStampTime-optitrackStampTime(1)));
[~,arEnd]   = min(abs(ardroneNavStampTime-optitrackStampTime(end)));
ardroneNavStampTime = ardroneNavStampTime(arStart:arEnd);
ardroneNavAltd      = ardroneNavAltd(arStart:arEnd);
ardroneNavVLin      = ardroneNavVLin(:,arStart:arEnd);
ardroneNavRot       = ardroneNavRot(:,arStart:arEnd);

% Select corresponding time frame in ardroneOdom data
[~,arStart] = min(abs(ardroneOdomStampTime-optitrackStampTime(1)));
[~,arEnd]   = min(abs(ardroneOdomStampTime-optitrackStampTime(end)));
ardroneOdomStampTime  = ardroneOdomStampTime(arStart:arEnd);
ardroneOdomPos        = ardroneOdomPos(:,arStart:arEnd);
ardroneOdomVLin       = ardroneOdomVLin(:,arStart:arEnd);
ardroneOdomOrientQuat = ardroneOdomOrientQuat(:,arStart:arEnd);
ardroneOdomVAng       = ardroneOdomVAng(:,arStart:arEnd);

% Ensure that all AR.Drone 2.0 signals have equal length by truncating
% Often the odometry are too short, so only check on these messages
% Correct Optitrack data too
nImu  = length(ardroneImuStampTime);
nNav  = length(ardroneNavStampTime);
nOdom = length(ardroneOdomStampTime);
if nOdom ~= nImu || nOdom ~= nNav
    % Select corresponding time frame in OptiTrack data
    [~,otEnd]   = min(abs(optitrackStampTime-ardroneOdomStampTime(end)));
    while optitrackStampTime(otEnd) < ardroneOdomStampTime(end)
        otEnd = otEnd + 1;
    end
    optitrackStampTime  = optitrackStampTime(1:otEnd);
    optitrackPos        = optitrackPos(:,1:otEnd);
    optitrackOrientQuat = optitrackOrientQuat(:,1:otEnd);

    % Select corresponding time frame in ardroneImu data
    [~,arEnd]   = min(abs(ardroneImuStampTime-ardroneOdomStampTime(end)));
    ardroneImuStampTime  = ardroneImuStampTime(1:arEnd);
    ardroneImuOrientQuat = ardroneImuOrientQuat(:,1:arEnd);
    ardroneImuVAng       = ardroneImuVAng(:,1:arEnd);

    % Select corresponding time frame in ardroneNav data
    [~,arEnd]   = min(abs(ardroneNavStampTime-ardroneOdomStampTime(end)));
    ardroneNavStampTime = ardroneNavStampTime(1:arEnd);
    ardroneNavAltd      = ardroneNavAltd(1:arEnd);
    ardroneNavVLin      = ardroneNavVLin(:,1:arEnd);
    ardroneNavRot       = ardroneNavRot(:,1:arEnd);
end

% Align time frames and start at 0
ardroneImuStampTime  = ardroneImuStampTime  - optitrackStampTime(1);
ardroneNavStampTime  = ardroneNavStampTime  - optitrackStampTime(1);
ardroneOdomStampTime = ardroneOdomStampTime - optitrackStampTime(1);
optitrackStampTime   = optitrackStampTime   - optitrackStampTime(1);


%% Convert quaternions to Euler angles
optitrackOrient = quat2EulAndWrap(optitrackOrientQuat,0);
ardroneImuOrient = quat2EulAndWrap(ardroneImuOrientQuat,0);
ardroneOdomOrient = quat2EulAndWrap(ardroneOdomOrientQuat,0);


%% TODO Zero data using average of first 9 seconds
tAvg = 9;

% Zero Optitrack data
[~,otAvgEnd] = min(abs(optitrackStampTime-tAvg));
optitrackPos = optitrackPos - mean(optitrackPos(:,1:otAvgEnd),2);
optitrackOrient = optitrackOrient - mean(optitrackOrient(:,1:otAvgEnd),2);

% Zero AR.Drone 2.0 IMU data
[~,arAvgEnd] = min(abs(ardroneImuStampTime-tAvg));
ardroneImuOrient = ardroneImuOrient - ...
                   mean(ardroneImuOrient(:,1:arAvgEnd),2);
ardroneImuVAng   = ardroneImuVAng - mean(ardroneImuVAng(:,1:arAvgEnd),2);

% Zero AR.Drone 2.0 navdata data
[~,arAvgEnd] = min(abs(ardroneNavStampTime-tAvg));
ardroneNavAltd = ardroneNavAltd - mean(ardroneNavAltd(:,1:arAvgEnd),2);
ardroneNavVLin = ardroneNavVLin - mean(ardroneNavVLin(:,1:arAvgEnd),2);
ardroneNavRot  = ardroneNavRot - mean(ardroneNavRot(:,1:arAvgEnd),2);

% Zero AR.Drone 2.0 odometry data
[~,arAvgEnd] = min(abs(ardroneOdomStampTime-tAvg));
ardroneOdomPos    = ardroneOdomPos - mean(ardroneOdomPos(:,1:arAvgEnd),2);
ardroneOdomVLin   = ardroneOdomVLin - ...
                    mean(ardroneOdomVLin(:,1:arAvgEnd),2);
ardroneOdomOrient = ardroneOdomOrient - ...
                    mean(ardroneOdomOrient(:,1:arAvgEnd),2);
ardroneOdomVAng   = ardroneOdomVAng - ...
                    mean(ardroneOdomVAng(:,1:arAvgEnd),2);


%% Edit Optitrack data


%% Edit AR.Drone 2.0 IMU data


%% Edit AR.Drone 2.0 navdata
ardroneNavAltd = ardroneNavAltd/1000;
ardroneNavVLin = ardroneNavVLin/1000;
ardroneNavRot  = unwrap(ardroneNavRot/180*pi);


%% Edit AR.Drone 2.0 odometry data


%% TODO Convert AR.Drone 2.0 on-board data to inertial frame
% For each sample in AR.Drone 2.0 data:
% - 
% nSamples = 
% for i = 1:
% Construct homogeneous transformation matrix (translational dynamics)
% R = zeros(4,4);
% R(1:3,1:3) = eul2rotm([psi,theta,phi]);
% R(1,4) = x;
% R(2,4) = y;
% R(3,4) = z;
% R(4,4) = 1;

% TODO: construct rotation matrix for rotational dynamics



%% Create plot styling settings
% set(0,'defaultAxesTickLabelInterpreter','latex');
% set(0,'defaultlegendinterpreter','latex');


%% Compare position sources
figure('Name','Position');
subplot(3,1,1);
plot(optitrackStampTime,optitrackPos(1,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomPos(1,:));
legend('Optitrack','AR.Drone 2.0 odometry');
title('x');
xlabel('Time (s)');
ylabel('x (m)');

subplot(3,1,2);
plot(optitrackStampTime,optitrackPos(2,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomPos(2,:));
legend('Optitrack','AR.Drone 2.0 odometry');
title('y');
xlabel('Time (s)');
ylabel('y (m)');

subplot(3,1,3);
plot(optitrackStampTime,optitrackPos(3,:));
hold on;
plot(ardroneNavStampTime,ardroneNavAltd);
plot(ardroneOdomStampTime,ardroneOdomPos(3,:));
legend('Optitrack','AR.Drone 2.0 navdata','AR.Drone 2.0 odometry');
title('z');
xlabel('Time (s)');
ylabel('z (m)');


%% Compare linear velocity sources
figure('Name','Linear velocity');
subplot(3,1,1);
plot(ardroneNavStampTime,ardroneNavVLin(1,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomVLin(1,:));
legend('AR.Drone 2.0 navdata','AR.Drone 2.0 odometry');
title('$\dot{x}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{x}$ (m/s)','Interpreter','latex');

subplot(3,1,2);
plot(ardroneNavStampTime,ardroneNavVLin(2,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomVLin(2,:));
legend('AR.Drone 2.0 navdata','AR.Drone 2.0 odometry');
title('$\dot{y}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{y}$ (m/s)','Interpreter','latex');

subplot(3,1,3);
plot(ardroneNavStampTime,ardroneNavVLin(3,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomVLin(3,:));
legend('AR.Drone 2.0 navdata','AR.Drone 2.0 odometry');
title('$\dot{z}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{z}$ (m/s)','Interpreter','latex');


%% Compare orientation sources
figure('Name','Orientation');
subplot(3,1,1);
plot(optitrackStampTime,optitrackOrient(2,:));
hold on;
plot(ardroneImuStampTime,ardroneImuOrient(1,:));
plot(ardroneNavStampTime,ardroneNavRot(1,:));
plot(ardroneOdomStampTime,ardroneOdomOrient(1,:));
legend('Optitrack','AR.Drone 2.0 IMU','AR.Drone 2.0 navdata',...
       'AR.Drone 2.0 odometry');
title('\phi');
xlabel('Time (s)');
ylabel('\phi (rad)');

subplot(3,1,2);
plot(optitrackStampTime,optitrackOrient(1,:));
hold on;
plot(ardroneImuStampTime,-ardroneImuOrient(2,:));
plot(ardroneNavStampTime,ardroneNavRot(2,:));
plot(ardroneOdomStampTime,-ardroneOdomOrient(2,:));
legend('Optitrack','AR.Drone 2.0 IMU','AR.Drone 2.0 navdata',...
       'AR.Drone 2.0 odometry');
title('\theta');
xlabel('Time (s)');
ylabel('\theta (rad)');

subplot(3,1,3);
plot(optitrackStampTime,optitrackOrient(3,:));
hold on;
plot(ardroneImuStampTime,ardroneImuOrient(3,:));
plot(ardroneNavStampTime,ardroneNavRot(3,:));
plot(ardroneOdomStampTime,ardroneOdomOrient(3,:));
legend('Optitrack','AR.Drone 2.0 IMU','AR.Drone 2.0 navdata',...
       'AR.Drone 2.0 odometry');
title('\psi');
xlabel('Time (s)');
ylabel('\psi (rad)');


%% Compare angular velocity sources
figure('Name','Angular velocity');
subplot(3,1,1);
plot(ardroneImuStampTime,ardroneImuVAng(1,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomVAng(1,:));
legend('AR.Drone 2.0 IMU','AR.Drone 2.0 odometry');
title('$\dot{\phi}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{\phi}$ (rad)','Interpreter','latex');

subplot(3,1,2);
plot(ardroneImuStampTime,ardroneImuVAng(2,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomVAng(2,:));
legend('AR.Drone 2.0 IMU','AR.Drone 2.0 odometry');
title('$\dot{\theta}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{\theta}$ (rad)','Interpreter','latex');

subplot(3,1,3);
plot(ardroneImuStampTime,ardroneImuVAng(3,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomVAng(3,:));
legend('AR.Drone 2.0 IMU','AR.Drone 2.0 odometry');
title('$\dot{\psi}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{\psi }$(rad)','Interpreter','latex');


%% Reset plot styling settings
% set(0,'defaultAxesTickLabelInterpreter','none');
% set(0,'defaultlegendinterpreter','none');
