%% Initialization
clear;
close all;
clc;


%% Set variables
% Retrieve bag file
cd ~/.ros;
bag = rosbag("force_torque_meas_2020-04-29-13-37-20.bag");
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

% Select topics that need to be stored
topics.cmdVel = 1;
topics.modelInput = 1;
topics.modelStates = 1;

% Set time interval with respect to start of rosbag recording
time = [8, 18];


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


%% Interpolate input data
data.time = modelInputTime;

data.value = force;
tmp = interpolate(0.001, data);
gazSim.input.time = tmp.time;
gazSim.input.force = tmp.value;

data.value = torque;
tmp = interpolate(0.001, data);
gazSim.input.torque = tmp.value;

% subplot(3,1,1);
% plot(modelInputTime, force(3,:), 'Marker', 'o');
% xlim([8.8 8.9]);
% ylim([0 30]);
% subplot(3,1,2);
% plot(gazSim.input.time, gazSim.input.force(3,:), 'Marker', 'o', ...
%      'Color', 'r');
% xlim([8.8 8.9]);
% ylim([0 30]);
% subplot(3,1,3);
% plot(modelInputTime, force(3,:), 'Marker', 'o');
% xlim([8.8 8.9]);
% ylim([0 30]);
% hold on;
% plot(gazSim.input.time, gazSim.input.force(3,:), 'Marker', 'o');
% hold off;
% legend('Original data', 'Interpolated data');


%% Interpolate states data
data.time = modelStatesTime;

data.value = pos;
tmp = interpolate(gazSim.input.time, data);
gazSim.state.time = tmp.time;
gazSim.state.pos = tmp.value;

data.value = orient;
tmp = interpolate(gazSim.input.time, data);
gazSim.state.orient = tmp.value;


%% Set start to 0, sample frequency = 1000
gazSim.input.time = gazSim.input.time - gazSim.input.time(1);
gazSim.state.time = gazSim.state.time - gazSim.state.time(1);

gazSim.sampleTime = 0.001;


%% Plot input data
figure('Name', 'Force plots');

subplot(3,1,1);
plot(gazSim.input.time, gazSim.input.force(1,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('f_x (N?)');

subplot(3,1,2);
plot(gazSim.input.time, gazSim.input.force(2,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('f_y (N?)');

subplot(3,1,3);
plot(gazSim.input.time, gazSim.input.force(3,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('f_z (N?)');


figure('Name', 'Torque plots');

subplot(3,1,1);
plot(gazSim.input.time, gazSim.input.torque(1,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('\tau_x (Nm?)');

subplot(3,1,2);
plot(gazSim.input.time, gazSim.input.torque(2,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('\tau_y (Nm?)');

subplot(3,1,3);
plot(gazSim.input.time, gazSim.input.torque(3,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('\tau_z (Nm?)');


%% Plot state data
figure('Name', 'Position (inertial frame)');

subplot(3,1,1);
plot(gazSim.state.time, gazSim.state.pos(1,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('x (m)');

subplot(3,1,2);
plot(gazSim.state.time, gazSim.state.pos(2,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('y (m)');

subplot(3,1,3);
plot(gazSim.state.time, gazSim.state.pos(3,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('z (m)');


%% Save gazSim data
save('gazSim.mat', 'gazSim');
