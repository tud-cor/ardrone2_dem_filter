%% Initialization
clear;
close all;
clc;


%% Set variables
% Retrieve bag file
cd ~/.ros;
bag = rosbag("ardrone2_exp_2020-07-24_2.bag");
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

% Select topics that need to be stored
% Controller signal topics
topics.cmdVel = 0;

% Simulation/flight data topics
topics.modelInput = 0;
topics.gazeboModelStates = 0;
topics.optitrack = 1;
topics.ardroneNavdata = 1;
topics.ardroneOdom = 0;
topics.rotorsMotorSpeed = 0;

% Set time interval with respect to start of rosbag recording
time = [41.8,60];


%% Get data
topicsOut = storeBagdata(bag, topics, time);

if topics.cmdVel
    cmdVelTime = topicsOut.cmdVel.time;
    cmdVelLin = topicsOut.cmdVel.lin;
    cmdVelAng = topicsOut.cmdVel.ang;
end

if topics.optitrack
    optitrackStampTime = topicsOut.optitrack.stampTime;
    optitrackRecordTime = topicsOut.optitrack.recordTime;
    optitrackPos = topicsOut.optitrack.pos;
    optitrackOrientQuat = topicsOut.optitrack.orient;
end

if topics.ardroneNavdata
    ardroneNavdataStampTime = topicsOut.ardroneNavdata.stampTime;
    ardroneNavdataRecordTime = topicsOut.ardroneNavdata.recordTime;
    ardroneNavdataMotor = topicsOut.ardroneNavdata.motor;
    ardroneNavdataRot = topicsOut.ardroneNavdata.rot;
    ardroneNavdataVLin = topicsOut.ardroneNavdata.vLin;
    ardroneNavdataALin = topicsOut.ardroneNavdata.aLin;
end

if topics.ardroneOdom
    ardroneOdomStampTime = topicsOut.ardroneOdom.stampTime;
    ardroneOdomRecordTime = topicsOut.ardroneOdom.recordTime;
    ardroneOdomPos = topicsOut.ardroneOdom.pos;
    ardroneOdomOrientQuat = topicsOut.ardroneOdom.orient;
    ardroneOdomVLin = topicsOut.ardroneOdom.vLin;
    ardroneOdomVAng = topicsOut.ardroneOdom.vAng;
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


%% Interpolate motor data
if topics.modelInput
    data.time = ardroneNavdataStampTime;

    data.value = ardroneNavdataMotor;
    tmp = interpolate(0.001, data);
    expData.input.time = tmp.time;
    expData.input.motor(1,:) = tmp.value;

    data.value = torque;
    tmp = interpolate(0.001, data);
    expData.input.torque = tmp.value;

    expData.input.time = expData.input.time - expData.input.time(1);
end


%% Save gazSim data
filename = sprintf('bagdata_%s', datestr(now,'dd-mm-yyyy_HH-MM'));
save(filename, 'expData');
