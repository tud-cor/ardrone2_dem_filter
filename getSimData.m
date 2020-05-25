%% Initialization
clear;
close all;
clc;


%% Set variables
% Retrieve bag file
cd ~/.ros;
% bag = rosbag("force_torque_meas_2020-05-13-09-08-59.bag");
bag = rosbag("force_torque_meas_2020-05-19-09-11-55.bag");
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

% Select topics that need to be stored
topics.cmdVel = 1;
topics.modelInput = 1;
topics.modelStates = 1;

% Set time interval with respect to start of rosbag recording
time = [18, 38];


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
    orientQuat = topicsOut.modelStates.orient;
    vLin = topicsOut.modelStates.vLin;
    vAng = topicsOut.modelStates.vAng;
end


%% Convert quaternions to ZYX Euler angles
if topics.modelStates
    orientQuat = orientQuat';
    orient = zeros(size(orientQuat,1),3);
    for i = 1:size(orientQuat,1)
        orient(i,:) = quat2eul(orientQuat(i,:));
    end
    orient = orient';

    % for i = 1:3
    %     % Check for jumps between ~0 and ~2*pi and center angles around 0.
    %     % If amount of jumps is at least equal to 50, the data will be edited
    %     cnt = 0;
    %     for j = 2:size(orient,2)
    %         if abs(orient(i,j) - orient(i,j-1)) > 6
    %             cnt = cnt + 1;
    %         end
    %     end
    %     if cnt >= 50
    %         orient(i,:) = orient(i,:) + 2*pi;
    %         orient(i,:) = mod(orient(i,:),2*pi);
    %         avg = mean(orient(i,:));
    %         orient(i,:) = orient(i,:) - avg;
    %         for j = 1:size(orient,2)
    %             if orient(i,j) < -0.75
    %                 orient(i,j) = 0;
    %             elseif orient(i,j) > 0.75
    %                 orient(i,j) = 0;
    %             end
    %         end
    %     end
    % end
end


%% Interpolate input data
if topics.modelInput
    data.time = modelInputTime;

    data.value = force;
    tmp = interpolate(0.001, data);
    gazSim.input.time = tmp.time;
    gazSim.input.force = tmp.value;

    data.value = torque;
    tmp = interpolate(0.001, data);
    gazSim.input.torque = tmp.value;
    
    gazSim.input.time = gazSim.input.time - gazSim.input.time(1);

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
end


%% Interpolate states data
if topics.modelStates
    data.time = modelStatesTime;

    data.value = pos;
    tmp = interpolate(gazSim.input.time, data);
    gazSim.state.time = tmp.time;
    gazSim.state.pos = tmp.value;

    data.value = orient;
    tmp = interpolate(gazSim.input.time, data);
    gazSim.state.orient = tmp.value;
    
    gazSim.state.time = gazSim.state.time - gazSim.state.time(1);
end


%% Set sample frequency to 1000 Hz
gazSim.sampleTime = 0.001;


%% Plot input data
figure('Name', 'Input plots');

subplot(3,2,1);
plot(gazSim.input.time, gazSim.input.force(1,:), 'LineStyle', '-', ...
     'Marker', '.');
title('Force');
xlabel('Time (s)');
ylabel('f_x (N?)');

subplot(3,2,3);
plot(gazSim.input.time, gazSim.input.force(2,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('f_y (N?)');

subplot(3,2,5);
plot(gazSim.input.time, gazSim.input.force(3,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('f_z (N?)');


subplot(3,2,2);
plot(gazSim.input.time, gazSim.input.torque(1,:), 'LineStyle', '-', ...
     'Marker', '.');
title('Torque');
xlabel('Time (s)');
ylabel('\tau_x (Nm?)');

subplot(3,2,4);
plot(gazSim.input.time, gazSim.input.torque(2,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('\tau_y (Nm?)');

subplot(3,2,6);
plot(gazSim.input.time, gazSim.input.torque(3,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('\tau_z (Nm?)');


%% Plot state data
figure('Name', 'State plots');

subplot(3,2,1);
plot(gazSim.state.time, gazSim.state.pos(1,:), 'LineStyle', '-', ...
     'Marker', '.');
title('Position (inertial frame)');
xlabel('Time (s)');
ylabel('x (m)');

subplot(3,2,3);
plot(gazSim.state.time, gazSim.state.pos(2,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('y (m)');

subplot(3,2,5);
plot(gazSim.state.time, gazSim.state.pos(3,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('z (m)');


subplot(3,2,2);
plot(gazSim.state.time, gazSim.state.orient(1,:), 'LineStyle', '-', ...
     'Marker', '.');
title('XYZ fixed/ZYX Euler angles');
xlabel('Time (s)');
ylabel('\phi (rad)');

subplot(3,2,4);
plot(gazSim.state.time, gazSim.state.orient(2,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('\theta (rad)');

subplot(3,2,6);
plot(gazSim.state.time, gazSim.state.orient(3,:), 'LineStyle', '-', ...
     'Marker', '.');
xlabel('Time (s)');
ylabel('\psi (rad)');

%% Save gazSim data
save('TODOChooseProperName.mat', 'gazSim');
