%% Initialization
clear;
close all;
clc;


%% Constant parameters
floatTol = 1e-6; %number with a smaller accuracy are treated as 0


%% Data to compare
% Possible state    | Available on topic
% --------------------------------------
% x                 |                   ardrone/odometry (drifting) TODO check with coordinate transforms
% y                 |                   ardrone/odometry (drifting) TODO check with coordinate transforms
% z                 | ardrone/navdata + ardrone/odometry (same, but a little bit of offset!) TODO check with coordinate transforms

% xDot              | ardrone/navdata + ardrone/odometry (same) TODO Check with Optitrack data for correctness
% yDot              | ardrone/navdata + ardrone/odometry (same) TODO Check with Optitrack data for correctness
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


%% Load data
load expData7_24_8.mat;

% Originally recorded data
origOtTime     = expData.origData.otTime;
origOtPos      = expData.origData.otPos;
origOtOrient   = expData.origData.otOrient;

origImuTime    = expData.origData.imuTime;
origImuOrient  = expData.origData.imuOrient;
origImuVAng    = expData.origData.imuVAng;

origNavTime    = expData.origData.navTime;
origNavAltd    = expData.origData.navAltd;
origNavVLin    = expData.origData.navVLin;
origNavRot     = expData.origData.navRot;

origOdomTime   = expData.origData.odomTime;
origOdomPos    = expData.origData.odomPos;
origOdomVLin   = expData.origData.odomVLin;
origOdomOrient = expData.origData.odomOrient;
origOdomVAng   = expData.origData.odomVAng;

% Interpolated data
t = expData.output.time;

otPos    = expData.output.otPos;
otOrient = expData.output.otOrient;

imuOrient = expData.output.imuOrient;
imuVAng   = expData.output.imuVAng;

navAltd = expData.output.navAltd;
navVLin = expData.output.navVLin;
navRot  = expData.output.navRot;

odomPos    = expData.output.odomPos;
odomVLin   = expData.output.odomVLin;
odomOrient = expData.output.odomOrient;
odomVAng   = expData.output.odomVAng;


%% Plot original data to determine how to subtract the offset from the
%  signals
% figure('Name','OptiTrack');
% subplot(2,1,1);
% plot(origOtTime,origOtPos(1,:));
% hold on;
% plot(origOtTime,origOtPos(2,:));
% plot(origOtTime,origOtPos(3,:));
% yline(0);
% legend('x','y','z','0 ref');
% subplot(2,1,2);
% plot(origOtTime,origOtOrient(1,:));
% hold on;
% plot(origOtTime,origOtOrient(2,:));
% plot(origOtTime,origOtOrient(3,:));
% yline(0);
% legend('\phi','\theta','\psi','0 ref');
% 
% 
% figure('Name','AR.Drone 2.0 IMU');
% subplot(2,1,1);
% plot(origImuTime,origImuOrient(1,:));
% hold on;
% plot(origImuTime,origImuOrient(2,:));
% plot(origImuTime,origImuOrient(3,:));
% yline(0);
% legend('\phi','\theta','\psi','0 ref');
% subplot(2,1,2);
% plot(origImuTime,origImuVAng(1,:));
% hold on;
% plot(origImuTime,origImuVAng(2,:));
% plot(origImuTime,origImuVAng(3,:));
% yline(0);
% legend('$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','0 ref',...
%        'Interpreter','latex');
% 
% 
% figure('Name','AR.Drone 2.0 navdata');
% subplot(3,1,1);
% plot(origNavTime,origNavAltd);
% yline(0);
% legend('z','0 ref');
% subplot(3,1,2);
% plot(origNavTime,origNavVLin(1,:));
% hold on;
% plot(origNavTime,origNavVLin(2,:));
% plot(origNavTime,origNavVLin(3,:));
% yline(0);
% legend('$\dot{x}$','$\dot{y}$','$\dot{z}$','0 ref',...
%        'Interpreter','latex');
% subplot(3,1,3);
% plot(origNavTime,origNavRot(1,:));
% hold on;
% plot(origNavTime,origNavRot(2,:));
% plot(origNavTime,origNavRot(3,:));
% yline(0);
% legend('\phi','\theta','\psi','0 ref');
% 
% 
% figure('Name','AR.Drone 2.0 odometry');
% subplot(4,1,1);
% plot(origOdomTime,origOdomPos(1,:));
% hold on;
% plot(origOdomTime,origOdomPos(2,:));
% plot(origOdomTime,origOdomPos(3,:));
% yline(0);
% legend('x','y','z','0 ref');
% subplot(4,1,2);
% plot(origOdomTime,origOdomVLin(1,:));
% hold on;
% plot(origOdomTime,origOdomVLin(2,:));
% plot(origOdomTime,origOdomVLin(3,:));
% yline(0);
% legend('$\dot{x}$','$\dot{y}$','$\dot{z}$','0 ref',...
%        'Interpreter','latex');
% subplot(4,1,3);
% plot(origOdomTime,origOdomOrient(1,:));
% hold on;
% plot(origOdomTime,origOdomOrient(2,:));
% plot(origOdomTime,origOdomOrient(3,:));
% yline(0);
% legend('\phi','\theta','\psi','0 ref');
% subplot(4,1,4);
% plot(origOdomTime,origOdomVAng(1,:));
% hold on;
% plot(origOdomTime,origOdomVAng(2,:));
% plot(origOdomTime,origOdomVAng(3,:));
% yline(0);
% legend('$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','0 ref',...
%        'Interpreter','latex');


%% Ensure proper offset and scaling of linear and angular quantities
% OptiTrack
tAvg = 3;
[~,otAvgEnd] = min(abs(origOtTime-tAvg));
otPos = otPos - mean(origOtPos(:,1:otAvgEnd),2);
otOrient(1:2,:) = otOrient(1:2,:) - mean(origOtOrient(1:2,1:otAvgEnd),2);
psiOffset = mean(otOrient(3,1:otAvgEnd));


%% Edit linear quantities (position/linear velocity)
navAltd = navAltd/1000;
navVLin = navVLin/1000;


%% Edit angular quantities (orientation/angular velocity)
% OptiTrack
otOrient = [0,-1,0;1,0,0;0,0,1]*otOrient + [0;0;pi/2];
otOrient = otOrient - otOrient(:,1);
otOrient = unwrap(otOrient);

% AR.Drone 2.0 IMU
imuOrient = [-1,0,0;0,-1,0;0,0,1]*imuOrient + [0;0;0];
imuOrient = imuOrient - imuOrient(:,1);
imuOrient = unwrap(imuOrient);

% AR.Drone 2.0 navdata
navOrient = navRot/180*pi;
navOrient = navOrient - navOrient(:,1);
navOrient = unwrap(navOrient);

% AR.Drone 2.0 odometry
odomOrient = [-1,0,0;0,-1,0;0,0,1]*odomOrient + [0;0;0];
odomOrient = odomOrient - odomOrient(:,1);
odomOrient = unwrap(odomOrient);


% %% TODO Zero data using average of first 9 seconds
% tAvg = 9;
% 
% % Zero Optitrack data
% [~,otAvgEnd] = min(abs(optitrackStampTime-tAvg));
% optitrackPos = optitrackPos - mean(optitrackPos(:,1:otAvgEnd),2);
% optitrackOrient = optitrackOrient - mean(optitrackOrient(:,1:otAvgEnd),2);
% 
% % Zero AR.Drone 2.0 IMU data
% [~,arAvgEnd] = min(abs(ardroneImuStampTime-tAvg));
% ardroneImuOrient = ardroneImuOrient - ...
%                    mean(ardroneImuOrient(:,1:arAvgEnd),2);
% ardroneImuVAng   = ardroneImuVAng - mean(ardroneImuVAng(:,1:arAvgEnd),2);
% 
% % Zero AR.Drone 2.0 navdata data
% [~,arAvgEnd] = min(abs(ardroneNavStampTime-tAvg));
% ardroneNavAltd = ardroneNavAltd - mean(ardroneNavAltd(:,1:arAvgEnd),2);
% ardroneNavVLin = ardroneNavVLin - mean(ardroneNavVLin(:,1:arAvgEnd),2);
% ardroneNavRot  = ardroneNavRot - mean(ardroneNavRot(:,1:arAvgEnd),2);
% 
% % Zero AR.Drone 2.0 odometry data
% [~,arAvgEnd] = min(abs(ardroneOdomStampTime-tAvg));
% ardroneOdomPos    = ardroneOdomPos - mean(ardroneOdomPos(:,1:arAvgEnd),2);
% ardroneOdomVLin   = ardroneOdomVLin - ...
%                     mean(ardroneOdomVLin(:,1:arAvgEnd),2);
% ardroneOdomOrient = ardroneOdomOrient - ...
%                     mean(ardroneOdomOrient(:,1:arAvgEnd),2);
% ardroneOdomVAng   = ardroneOdomVAng - ...
%                     mean(ardroneOdomVAng(:,1:arAvgEnd),2);


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


%% Compare position, linear velocity, orientation and angular velocity of
%  OptiTrack and different sensors in AR.Drone 2.0
figure('Name','Position');
subplot(3,1,1);
plot(t,otPos(1,:));
hold on;
plot(t,odomPos(1,:));
legend('Optitrack','AR.Drone 2.0 odometry');
title('x');
xlabel('Time (s)');
ylabel('x (m)');

subplot(3,1,2);
plot(t,otPos(2,:));
hold on;
plot(t,odomPos(2,:));
legend('Optitrack','AR.Drone 2.0 odometry');
title('y');
xlabel('Time (s)');
ylabel('y (m)');

subplot(3,1,3);
plot(t,otPos(3,:));
hold on;
plot(t,navAltd);
plot(t,odomPos(3,:));
legend('Optitrack','AR.Drone 2.0 navdata','AR.Drone 2.0 odometry');
title('z');
xlabel('Time (s)');
ylabel('z (m)');


figure('Name','Linear velocity');
subplot(3,1,1);
plot(t,navVLin(1,:));
hold on;
plot(t,odomVLin(1,:));
legend('AR.Drone 2.0 navdata','AR.Drone 2.0 odometry');
title('$\dot{x}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{x}$ (m/s)','Interpreter','latex');

subplot(3,1,2);
plot(t,navVLin(2,:));
hold on;
plot(t,odomVLin(2,:));
legend('AR.Drone 2.0 navdata','AR.Drone 2.0 odometry');
title('$\dot{y}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{y}$ (m/s)','Interpreter','latex');

subplot(3,1,3);
plot(t,navVLin(3,:));
hold on;
plot(t,odomVLin(3,:));
legend('AR.Drone 2.0 navdata','AR.Drone 2.0 odometry');
title('$\dot{z}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{z}$ (m/s)','Interpreter','latex');


figure('Name','Orientation');
subplot(3,1,1);
plot(t,otOrient(1,:));
hold on;
plot(t,imuOrient(1,:));
plot(t,navOrient(1,:));
plot(t,odomOrient(1,:));
legend('Optitrack','AR.Drone 2.0 IMU','AR.Drone 2.0 navdata',...
       'AR.Drone 2.0 odometry');
title('\phi');
xlabel('Time (s)');
ylabel('\phi (rad)');

subplot(3,1,2);
plot(t,otOrient(2,:));
hold on;
plot(t,imuOrient(2,:));
plot(t,navOrient(2,:));
plot(t,odomOrient(2,:));
legend('Optitrack','AR.Drone 2.0 IMU','AR.Drone 2.0 navdata',...
       'AR.Drone 2.0 odometry');
title('\theta');
xlabel('Time (s)');
ylabel('\theta (rad)');

subplot(3,1,3);
plot(t,otOrient(3,:));
hold on;
plot(t,imuOrient(3,:));
plot(t,navOrient(3,:));
plot(t,odomOrient(3,:));
legend('Optitrack','AR.Drone 2.0 IMU','AR.Drone 2.0 navdata',...
       'AR.Drone 2.0 odometry');
title('\psi');
xlabel('Time (s)');
ylabel('\psi (rad)');


figure('Name','Angular velocity');
subplot(3,1,1);
plot(t,imuVAng(1,:));
hold on;
plot(t,odomVAng(1,:));
legend('AR.Drone 2.0 IMU','AR.Drone 2.0 odometry');
title('$\dot{\phi}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{\phi}$ (rad)','Interpreter','latex');

subplot(3,1,2);
plot(t,imuVAng(2,:));
hold on;
plot(t,odomVAng(2,:));
legend('AR.Drone 2.0 IMU','AR.Drone 2.0 odometry');
title('$\dot{\theta}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{\theta}$ (rad)','Interpreter','latex');

subplot(3,1,3);
plot(t,imuVAng(3,:));
hold on;
plot(t,odomVAng(3,:));
legend('AR.Drone 2.0 IMU','AR.Drone 2.0 odometry');
title('$\dot{\psi}$','Interpreter','latex');
xlabel('Time (s)');
ylabel('$\dot{\psi }$(rad)','Interpreter','latex');

