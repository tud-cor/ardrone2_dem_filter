%% Initialization
clear;
close all;
clc;


%% Load quaternion data
% Retrieve bag file
cd ~/.ros;
bag = rosbag("ardrone2_exp_2020-07-24_3.bag");
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
topics.ardroneOdom = 1;
topics.rotorsMotorSpeed = 0;

% Set time interval with respect to start of rosbag recording
time = [140,180];


%% Get data
topicsOut = storeBagdata(bag,topics,time);

if topics.optitrack
    optitrackStampTime  = topicsOut.optitrack.stampTime;
    optitrackPos        = topicsOut.optitrack.pos;
end

if topics.ardroneOdom
    ardroneOdomStampTime  = topicsOut.ardroneOdom.stampTime;
    ardroneOdomVLin       = topicsOut.ardroneOdom.vLin;
    ardroneOdomOrientQuat = topicsOut.ardroneOdom.orient;
end
ardroneOdomStampTime = ardroneOdomStampTime - optitrackStampTime(1);

optitrackStampTime = optitrackStampTime - optitrackStampTime(1);


%% Convert to Euler angles
ardroneOdomOrient = quat2EulAndWrap(ardroneOdomOrientQuat,0);


%% Save data for code speedup
save('checkARDrone2AnglesExp7_24_3.mat',...
     'optitrackStampTime','optitrackPos',...
     'ardroneOdomStampTime','ardroneOdomVLin','ardroneOdomOrient');


%% Load data for code speedup
load checkARDrone2AnglesExp7_24_3.mat;


%% Correct angle data
% phi_c = -phi_i
ardroneOdomOrientC(1,:) = -ardroneOdomOrient(1,:);

% theta_c = -theta_i
ardroneOdomOrientC(2,:) = -ardroneOdomOrient(2,:);

% New psi is old psi + pi/2
ardroneOdomOrientC(3,:) = ardroneOdomOrient(3,:);

% Zero angle data
ardroneOdomOrientC = ardroneOdomOrientC - ardroneOdomOrientC(:,1);


%% Zero angle data
ardroneOdomOrient = ardroneOdomOrient - ardroneOdomOrient(:,1);


%% Plot for imu/odom
figure('Name','Position and orientation');
subplot(3,1,1);
plot(optitrackStampTime,optitrackPos(1,:));
hold on;
plot(optitrackStampTime,optitrackPos(2,:));
plot(optitrackStampTime,optitrackPos(3,:));
yline(0);
legend('x','y','z','0 ref');

% figure('Name','Linear velocity');
% plot(ardroneOdomStampTime,ardroneOdomVLin(1,:));
% hold on;
% plot(ardroneOdomStampTime,ardroneOdomVLin(2,:));
% yline(0);
% legend('v_x','v_y','0 ref');

% figure('Name','Original orientation');
subplot(3,1,2);
plot(ardroneOdomStampTime,ardroneOdomOrient(1,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomOrient(2,:));
plot(ardroneOdomStampTime,ardroneOdomOrient(3,:));
yline(0);
legend('\phi_i','\theta_i','\psi_i','0 ref');

% figure('Name','Corrected orientation');
subplot(3,1,3);
plot(ardroneOdomStampTime,ardroneOdomOrientC(1,:));
hold on;
plot(ardroneOdomStampTime,ardroneOdomOrientC(2,:));
plot(ardroneOdomStampTime,ardroneOdomOrientC(3,:));
yline(0);
legend('\phi_c','\theta_c','\psi_c','0 ref');


%% Plot for navdata
