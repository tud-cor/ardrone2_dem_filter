%% Initialization
clear;
close all;
clc;


%% Set variables
% Retrieve bag file
cd ~/.ros;
bag = rosbag("ardrone2_exp_2020-07-24_7.bag");
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

% Select topics that need to be stored
% Controller signal topics
topics.cmdVel = 0;

% Simulation/flight data topics
topics.modelInput = 0;
topics.gazeboModelStates = 0;
topics.optitrack = 1;
topics.ardroneNavdata = 0;
topics.ardroneOdom = 1;
topics.rotorsMotorSpeed = 0;

% Set time interval with respect to start of rosbag recording
time = [20,120];


%% Get data
topicsOut = storeBagdata(bag, topics, time);

if topics.optitrack
    optitrackStampTime = topicsOut.optitrack.stampTime;
    optitrackRecordTime = topicsOut.optitrack.recordTime;
    optitrackPos = topicsOut.optitrack.pos;
    optitrackOrientQuat = topicsOut.optitrack.orient;
end

if topics.ardroneOdom
    ardroneOdomStampTime = topicsOut.ardroneOdom.stampTime;
    ardroneOdomRecordTime = topicsOut.ardroneOdom.recordTime;
    ardroneOdomPos = topicsOut.ardroneOdom.pos;
    ardroneOdomOrientQuat = topicsOut.ardroneOdom.orient;
    ardroneOdomVLin = topicsOut.ardroneOdom.vLin;
    ardroneOdomVAng = topicsOut.ardroneOdom.vAng;
end


%% Take derivative of OptiTrack data
vLen = size(optitrackPos,2) - 1;
optitrackVLin = zeros(3,vLen);
for i = 1:vLen
    optitrackVLin(:,i) = (optitrackPos(:,i+1) - optitrackPos(:,i))/...
                         (optitrackStampTime(i+1) - optitrackStampTime(i));
end


%% TEMP
optitrackPosZeroed = optitrackPos - optitrackPos(:,1);
ardroneOdomStampTimeZeroed = ardroneOdomStampTime - optitrackStampTime(1);
optitrackStampTimeZeroed = optitrackStampTime - optitrackStampTime(1);
save('optitrackDetermineVxs','ardroneOdomStampTime','ardroneOdomStampTimeZeroed','ardroneOdomRecordTime','ardroneOdomPos','ardroneOdomOrientQuat','ardroneOdomVAng','ardroneOdomVLin','optitrackOrientQuat','optitrackPos','optitrackStampTime','optitrackStampTimeZeroed','optitrackPosZeroed');


%% Plot odometry data
ardroneOdomStampTime = ardroneOdomStampTime - ardroneOdomStampTime(1);
ardroneOdomRecordTime = ardroneOdomRecordTime - ardroneOdomRecordTime(1);
figure('Name', 'AR.Drone 2.0 odometry plots');
sgtitle('Difference between recorded time and stamped time','FontSize',20);

subplot(3,2,1);
plot(ardroneOdomStampTime, ardroneOdomPos(1,:), '-o');
% plot(ardroneOdomStampTime(3166:3356), ardroneOdomPos(1,3166:3356), '-o');
title('Odometry position (stamped time)');
xlabel('Time (s)');
ylabel('x (m)');
xlim([0,40]);
% xlim([17.0,18.4]);

subplot(3,2,3);
plot(ardroneOdomStampTime, ardroneOdomPos(2,:), '-o');
% plot(ardroneOdomStampTime(3166:3356), ardroneOdomPos(2,3166:3356), '-o');
xlabel('Time (s)');
ylabel('y (m)');
xlim([0,40]);
% xlim([17.0,18.4]);

subplot(3,2,5);
plot(ardroneOdomStampTime, ardroneOdomPos(3,:), '-o');
% plot(ardroneOdomStampTime(3166:3356), ardroneOdomPos(3,3166:3356), '-o');
xlabel('Time (s)');
ylabel('z (m)');
xlim([0,40]);
% xlim([17.0,18.4]);


subplot(3,2,2);
plot(ardroneOdomRecordTime, ardroneOdomPos(1,:), '-o');
% plot(ardroneOdomRecordTime(3166:3356), ardroneOdomPos(1,3166:3356), '-o');
title('Odometry position (recorded time)');
xlabel('Time (s)');
ylabel('x (m)');

subplot(3,2,4);
plot(ardroneOdomRecordTime, ardroneOdomPos(2,:), '-o');
% plot(ardroneOdomRecordTime(3166:3356), ardroneOdomPos(2,3166:3356), '-o');
xlabel('Time (s)');
ylabel('y (m)');

subplot(3,2,6);
plot(ardroneOdomRecordTime, ardroneOdomPos(3,:), '-o');
% plot(ardroneOdomRecordTime(3166:3356), ardroneOdomPos(3,3166:3356), '-o');
xlabel('Time (s)');
ylabel('z (m)');