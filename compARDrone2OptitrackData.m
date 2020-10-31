%% Initialization
clear;
close all;
clc;


%% Constant parameters
floatTol = 1e-6; %number with a smaller accuracy are treated as 0
g        = 9.81; %gravitational constant
rad2deg  = 180/(2*pi);


%% Data to compare
% Possible state    | Available on OptiTrack topic
% ------------------------------------------------
% x                 | ardrone2/pose
% y                 | ardrone2/pose
% z                 | ardrone2/pose

% xDot              | -
% yDot              | -
% zDot              | -

% xDDot             | -
% yDDot             | -
% zDDot             | -

% phi               | ardrone2/pose
% theta             | ardrone2/pose
% psi               | ardrone2/pose

% phiDot            | -
% thetaDot          | -
% psiDot            | -


% Possible state    | Available on AR.Drone 2.0 topic
% ---------------------------------------------------
% x                 |                   ardrone/odometry TODO Check convention
% y                 |                   ardrone/odometry TODO Check convention
% z                 | ardrone/navdata + ardrone/odometry (same, but a little bit of offset!) TODO Check convention

% xDot              | ardrone/navdata + ardrone/odometry (same) TODO Check with Optitrack data for correctness
% yDot              | ardrone/navdata + ardrone/odometry (same) TODO Check with Optitrack data for correctness
% zDot              | ardrone/navdata + ardrone/odometry (same, but 0!)

% xDDot             | ardrone/imu + ardrone/navdata (orig: [m/s^2,g], but processed) Body frame
% yDDot             | ardrone/imu + ardrone/navdata (orig: [m/s^2,g], but processed) Body frame
% zDDot             | ardrone/imu + ardrone/navdata (orig: [m/s^2,g], but processed) Body frame

% phi               | ardrone/imu + ardrone/navdata + ardrone/odometry TODO check convention
% theta             | ardrone/imu + ardrone/navdata + ardrone/odometry TODO check convention
% psi               | ardrone/imu + ardrone/navdata + ardrone/odometry(2x) TODO check convention

% phiDot            | ardrone/imu + ardrone/odometry (odom is 0!)
% thetaDot          | ardrone/imu + ardrone/odometry (odom is 0!)
% psiDot            | ardrone/imu + ardrone/odometry (odom is 0!)


%% Data to be used to run the filters
% Possible state    | Chosen topic
% --------------------------------
% x                 | ardrone2/pose (inertial frame)
% y                 | ardrone2/pose (inertial frame)
% z                 | -

% xDot              | ardrone/navdata (body frame?)
% yDot              | ardrone/navdata (body frame?)
% zDot              | -

% xDDot             | ardrone/imu (body frame)
% yDDot             | ardrone/imu (body frame)
% zDDot             | -

% phi               | ardrone2/pose (inertial frame)
% theta             | ardrone2/pose (inertial frame)
% psi               | -

% phiDot            | ardrone/imu (body frame)
% thetaDot          | ardrone/imu (body frame)
% psiDot            | -


%% Load data
load expData10_29_25.mat;

% Originally recorded data
origOtTime     = expData.origData.otTime;
origOtPos      = expData.origData.otPos;
origOtOrient   = expData.origData.otOrient;

origImuTime    = expData.origData.imuTime;
origImuALin    = expData.origData.imuALin;
origImuOrient  = expData.origData.imuOrient;
origImuVAng    = expData.origData.imuVAng;

origNavTime    = expData.origData.navTime;
origNavAltd    = expData.origData.navAltd;
origNavVLin    = expData.origData.navVLin;
origNavALin    = expData.origData.navALin;
origNavOrient  = expData.origData.navOrient;

origOdomTime   = expData.origData.odomTime;
origOdomPos    = expData.origData.odomPos;
origOdomVLin   = expData.origData.odomVLin;
origOdomOrient = expData.origData.odomOrient;
origOdomVAng   = expData.origData.odomVAng;

% Interpolated data
t = expData.output.time;

otPos    = expData.output.otPos;
otOrient = expData.output.otOrient;

imuALin   = expData.output.imuALin;
imuOrient = expData.output.imuOrient;
imuVAng   = expData.output.imuVAng;

navAltd   = expData.output.navAltd;
navVLin   = expData.output.navVLin;
navALin   = expData.output.navALin;
navOrient = expData.output.navOrient;

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
% legend('\psi','\theta','\phi','0 ref');
% 
% 
% figure('Name','AR.Drone 2.0 IMU');
% subplot(3,1,1);
% plot(origImuTime,origImuALin(1,:));
% hold on;
% plot(origImuTime,origImuALin(2,:));
% plot(origImuTime,origImuALin(3,:));
% yline(0);
% legend('$\ddot{x}$','$\ddot{y}$','$\ddot{z}$','0 ref',...
%        'Interpreter','latex');
% subplot(3,1,2);
% plot(origImuTime,origImuOrient(1,:));
% hold on;
% plot(origImuTime,origImuOrient(2,:));
% plot(origImuTime,origImuOrient(3,:));
% yline(0);
% legend('\phi','\theta','\psi','0 ref');
% subplot(3,1,3);
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
% subplot(4,1,1);
% plot(origNavTime,origNavAltd);
% yline(0);
% legend('z','0 ref');
% subplot(4,1,2);
% plot(origNavTime,origNavVLin(1,:));
% hold on;
% plot(origNavTime,origNavVLin(2,:));
% plot(origNavTime,origNavVLin(3,:));
% yline(0);
% legend('$\dot{x}$','$\dot{y}$','$\dot{z}$','0 ref',...
%        'Interpreter','latex');
% subplot(4,1,3);
% plot(origNavTime,origNavALin(1,:));
% hold on;
% plot(origNavTime,origNavALin(2,:));
% plot(origNavTime,origNavALin(3,:));
% yline(0);
% legend('$\ddot{x}$','$\ddot{y}$','$\ddot{z}$','0 ref',...
%        'Interpreter','latex');
% subplot(4,1,4);
% plot(origNavTime,origNavOrient(1,:));
% hold on;
% plot(origNavTime,origNavOrient(2,:));
% plot(origNavTime,origNavOrient(3,:));
% yline(0);
% legend('\psi','\theta','\phi','0 ref');
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
% legend('\psi','\theta','\phi','0 ref');
% subplot(4,1,4);
% plot(origOdomTime,origOdomVAng(1,:));
% hold on;
% plot(origOdomTime,origOdomVAng(2,:));
% plot(origOdomTime,origOdomVAng(3,:));
% yline(0);
% legend('$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','0 ref',...
%        'Interpreter','latex');


%% Convert AR.Drone 2.0 on-board data to inertial frame
% For each sample in AR.Drone 2.0 data:
% - Convert linear coordinates from body frame to inertial frame
% - Convert angular coordinates from body frame to inertial frame

nSamples = length(expData.output.time);

navVLinI          = zeros(3,nSamples);
origImuALinOffset = origImuALin(:,1) - [0;0;g];
imuALinComp       = zeros(3,nSamples);
imuALinComp2      = zeros(3,nSamples);
imuALinI          = zeros(3,nSamples);

imuVAngI  = zeros(3,nSamples);
odomVAngI = zeros(3,nSamples);

for i = 1:nSamples
% Get current orientation
psi   = otOrient(1,i);
theta = otOrient(2,i);
phi   = otOrient(3,i);

% Construct homogeneous transformation matrix (translational dynamics)
% RBI: convert coordinates expressed in B to coordinates expressed in I
RBI = eul2rotm([psi,theta,phi],'ZYX');

RBICheck = zeros(3,3);
RBICheck(1,1) = cos(theta)*cos(psi);
RBICheck(1,2) = sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
RBICheck(1,3) = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
RBICheck(2,1) = cos(theta)*sin(psi);
RBICheck(2,2) = sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
RBICheck(2,3) = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
RBICheck(3,1) = -sin(theta);
RBICheck(3,2) = sin(phi)*cos(theta);
RBICheck(3,3) = cos(phi)*cos(theta);

RIB = RBI';


% Construct rotation matrix for rotational dynamics
% RrBI: convert ZYX Euler angles expressed in B to ZYX Euler angles
%       expressed in I
% RrIB: convert ZYX Euler angles expressed in I to ZYX Euler angles
%       expressed in B
RrBI = zeros(3,3);
RrBI(1,1) = 1;
RrBI(1,2) = sin(phi)*tan(theta);
RrBI(1,3) = cos(phi)*tan(theta);
RrBI(2,2) = cos(phi);
RrBI(2,3) = -sin(phi);
RrBI(3,2) = sin(phi)/cos(theta);
RrBI(3,3) = cos(phi)/cos(theta);

RrIB = zeros(3,3);
RrIB(1,1) = 1;
RrIB(1,3) = -sin(theta);
RrIB(2,2) = cos(phi);
RrIB(2,3) = sin(phi)*cos(theta);
RrIB(3,2) = -sin(phi);
RrIB(3,3) = cos(phi)*cos(theta);

RrBICheck = inv(RrIB);


% Compensate for gravity effects in accelerometer data
% Assumption 1: no coriolis effects
% Assumption 2: IMU is placed with perfect orientation in drone
imuALinComp(:,i) = imuALin(:,1) - RIB*[0;0;g];
imuALinComp2(:,i) = imuALinComp(:,i) - origImuALinOffset;


% Express linear quantities in I
% TODO: check if velocity data is already in inertial frame - probably yes!
navVLinI(:,i) = rotz(psi*rad2deg)*navVLin(:,i);
imuALinI(:,i) = RBI*imuALinComp2(:,i);


% Express angular quantities in I
imuVAngI(:,i)  = RrBI*imuVAng(:,i);
end


%% Compare position, linear velocity and linear acceleration data
figure('Name','Linear quantities');
subplot(3,1,1);
plot(t,otPos);
hold on;
yline(0);
legend('x','y','z','0 ref');
xlabel('Time (s)');
ylabel('Position (m)');
title('Linear quantities');
subplot(3,1,2);
plot(t,navVLinI);
hold on;
yline(0);
legend('$\dot{x}$','$\dot{y}$','$\dot{z}$','0 ref','Interpreter','latex');
xlabel('Time (s)');
ylabel('Linear velocity (m/s)');
subplot(3,1,3);
plot(t,imuALinI);
hold on;
yline(0);
legend('$\ddot{x}$','$\ddot{y}$','$\ddot{z}$','0 ref',...
       'Interpreter','latex');
xlabel('Time (s)');
ylabel('Linear acceleration (m/s^2)');


%% Compare orientation and angular velocity data
figure('Name','Angular quantities');
subplot(2,1,1);
plot(t,otOrient);
hold on;
yline(0);
legend('\psi','\theta','\phi','0 ref');
xlabel('Time (s)');
ylabel('Orientation using ZYX Euler angles (rad)');
title('Angular quantities');
subplot(2,1,2);
plot(t,imuVAngI);
hold on;
yline(0);
legend('$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','0 ref',...
       'Interpreter','latex');
xlabel('Time (s)');
ylabel('Angular velocity (rad/s)');

% subplot(3,1,1);
% plot(t,otOrient(1,:));
% hold on;
% plot(t,imuOrient(1,:));
% plot(t,navOrient(1,:));
% plot(t,odomOrient(1,:));
% legend('Optitrack','AR.Drone 2.0 IMU','AR.Drone 2.0 navdata',...
%        'AR.Drone 2.0 odometry');
% title('\psi');
% xlabel('Time (s)');
% ylabel('\psi (rad)');
% 
% subplot(3,1,2);
% plot(t,otOrient(2,:));
% hold on;
% plot(t,imuOrient(2,:));
% plot(t,navOrient(2,:));
% plot(t,odomOrient(2,:));
% legend('Optitrack','AR.Drone 2.0 IMU','AR.Drone 2.0 navdata',...
%        'AR.Drone 2.0 odometry');
% title('\theta');
% xlabel('Time (s)');
% ylabel('\theta (rad)');
% 
% subplot(3,1,3);
% plot(t,otOrient(3,:));
% hold on;
% plot(t,imuOrient(3,:));
% plot(t,navOrient(3,:));
% plot(t,odomOrient(3,:));
% legend('Optitrack','AR.Drone 2.0 IMU','AR.Drone 2.0 navdata',...
%        'AR.Drone 2.0 odometry');
% title('\phi');
% xlabel('Time (s)');
% ylabel('\phi (rad)');
% 
% 
% figure('Name','Angular velocity');
% subplot(3,1,1);
% plot(t,imuVAng(1,:));
% hold on;
% plot(t,odomVAng(1,:));
% legend('AR.Drone 2.0 IMU','AR.Drone 2.0 odometry');
% title('$\dot{\phi}$','Interpreter','latex');
% xlabel('Time (s)');
% ylabel('$\dot{\phi}$ (rad)','Interpreter','latex');
% 
% subplot(3,1,2);
% plot(t,imuVAng(2,:));
% hold on;
% plot(t,odomVAng(2,:));
% legend('AR.Drone 2.0 IMU','AR.Drone 2.0 odometry');
% title('$\dot{\theta}$','Interpreter','latex');
% xlabel('Time (s)');
% ylabel('$\dot{\theta}$ (rad)','Interpreter','latex');
% 
% subplot(3,1,3);
% plot(t,imuVAng(3,:));
% hold on;
% plot(t,odomVAng(3,:));
% legend('AR.Drone 2.0 IMU','AR.Drone 2.0 odometry');
% title('$\dot{\psi}$','Interpreter','latex');
% xlabel('Time (s)');
% ylabel('$\dot{\psi }$(rad)','Interpreter','latex');
% 
