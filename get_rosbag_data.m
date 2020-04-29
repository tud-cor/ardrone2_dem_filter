%% Initialization
clear;
close all;
clc;


%% Set variables
% Retrieve bag file
cd ~/.ros;
bag = rosbag("force_torque_meas_2020-04-29-13-37-20.bag");
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

topics.cmdVel = 1;
topics.modelInput = 1;
topics.modelStates = 1;

time = [3, 10];


%% Get data
topicsOut = storeBagdata(bag, topics, time);

if topics.cmdVel
    cmdVelTime = topicsOut.cmdVel.time;
    cmdVelLin = topicsOut.cmdVel.lin;
    cmdVelAng = topicsOut.cmdVel.ang;
end

if topics.modelInput
    modelInputTime = topicsOut.modelInput.time;
    force = topicsOut.modelInput.force;
    torque = topicsOut.modelInput.torque;
end

if topics.modelStates
    modelStatesTime = topicsOut.modelStates.time;
    pos = topicsOut.modelStates.pos;
    orient = topicsOut.modelStates.orient;
    vLin = topicsOut.modelStates.vLin;
    vAng = topicsOut.modelStates.vAng;
end


%% Plot data
figure('Name', 'Force plots');

subplot(3,1,1);
plot(modelInputTime, force(1,:), 'LineStyle', '-', 'Marker', '.');
title('f_x');
xlabel('Time (s)');
ylabel('Force (N?)');

subplot(3,1,2);
plot(modelInputTime, force(2,:), 'LineStyle', '-', 'Marker', '.');
title('f_y');
xlabel('Time (s)');
ylabel('Force (N?)');

subplot(3,1,3);
plot(modelInputTime, force(3,:), 'LineStyle', '-', 'Marker', '.');
title('f_z');
xlabel('Time (s)');
ylabel('Force (N?)');


figure('Name', 'Torque plots');

subplot(3,1,1);
plot(modelInputTime, torque(1,:), 'LineStyle', '-', 'Marker', '.');
title('\tau_x');
xlabel('Time (s)');
ylabel('Torque (Nm?)');

subplot(3,1,2);
plot(modelInputTime, torque(2,:), 'LineStyle', '-', 'Marker', '.');
title('\tau_y');
xlabel('Time (s)');
ylabel('Torque (Nm?)');

subplot(3,1,3);
plot(modelInputTime, torque(3,:), 'LineStyle', '-', 'Marker', '.');
title('\tau_z');
xlabel('Time (s)');
ylabel('Torque (Nm?)');