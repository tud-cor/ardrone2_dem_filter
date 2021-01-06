%% Initialization
clear;
close all;
clc;


%% Set variables
% Retrieve bag file
cd ~/.ros;
bag = rosbag("tum_ardrone_controller.bag");
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

% Select topics that need to be stored
% Controller signal topics
topics.cmdVel = 0;

% Simulation/flight data topics
topics.modelInput = 1;
topics.gazeboModelStates = 1;
topics.optitrack = 0;
topics.ardroneNavdata = 0;
topics.ardroneOdom = 0;
topics.rotorsMotorSpeed = 0;

% Set time interval with respect to start of rosbag recording
time = [10,20];


%% Get data
topicsOut = storeBagdata(bag, topics, time);

if topics.cmdVel
    cmdVelTime = topicsOut.cmdVel.time;
    cmdVelLin = topicsOut.cmdVel.lin;
    cmdVelAng = topicsOut.cmdVel.ang;
end

if topics.modelInput
    modelInputStampTime = topicsOut.modelInput.stampTime;
    modelInputRecordTime = topicsOut.modelInput.recordTime;
    force = topicsOut.modelInput.force;
    torque = topicsOut.modelInput.torque;
end

if topics.gazeboModelStates
    gazeboModelStatesTime = topicsOut.gazeboModelStates.time;
    gazeboPos = topicsOut.gazeboModelStates.pos;
    gazeboOrientQuat = topicsOut.gazeboModelStates.orient;
    gazeboVLin = topicsOut.gazeboModelStates.vLin;
    gazeboVAng = topicsOut.gazeboModelStates.vAng;
end

if topics.ardroneOdom
    ardroneOdomStampTime = topicsOut.ardroneOdom.stampTime;
    ardroneOdomRecordTime = topicsOut.ardroneOdom.recordTime;
    ardroneOdomPos = topicsOut.ardroneOdom.pos;
    ardroneOdomOrientQuat = topicsOut.ardroneOdom.orient;
    ardroneOdomVLin = topicsOut.ardroneOdom.vLin;
    ardroneOdomVAng = topicsOut.ardroneOdom.vAng;
end

if topics.ardroneNavdata
    ardroneNavdataStampTime = topicsOut.ardroneNavdata.stampTime;
    ardroneNavdataRecordTime = topicsOut.ardroneNavdata.recordTime;
    ardroneNavdataMotor = topicsOut.ardroneNavdata.motor;
    ardroneNavdataRot = topicsOut.ardroneNavdata.rot;
    ardroneNavdataVLin = topicsOut.ardroneNavdata.vLin;
    ardroneNavdataALin = topicsOut.ardroneNavdata.aLin;
end

if topics.rotorsMotorSpeed
    rotorTime = topicsOut.motorSpeed.time;
    rotorAngVel = topicsOut.motorSpeed.angVel;
end


%% Convert quaternions to ZYX Euler angles
if topics.gazeboModelStates
    gazeboOrientQuat = gazeboOrientQuat';
    orient = zeros(size(gazeboOrientQuat,1),3);
    for i = 1:size(gazeboOrientQuat,1)
        orient(i,:) = quat2eul(gazeboOrientQuat(i,:));
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
    data.time = modelInputStampTime;

    data.value = force;
    tmp = interpolate(0.001, data);
    gazSim.input.time = tmp.time;
    gazSim.input.force = tmp.value;

    data.value = torque;
    tmp = interpolate(0.001, data);
    gazSim.input.torque = tmp.value;
end


%% Interpolate states data
if topics.gazeboModelStates
    data.time = gazeboModelStatesTime;

    data.value = gazeboPos;
    tmp = interpolate(gazSim.input.time, data);
    gazSim.state.time = tmp.time;
    gazSim.state.pos = tmp.value;

    data.value = orient;
    tmp = interpolate(gazSim.input.time, data);
    gazSim.state.orient = tmp.value;
end


%% Correctly initialize time data
gazSim.state.time = gazSim.state.time - gazSim.state.time(1);


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
filename = sprintf('bagdata_%s', datestr(now,'dd-mm-yyyy_HH-MM'));
save(filename, 'gazSim');
