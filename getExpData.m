%% Initialization
clear;
close all;
clc;

% Parameters
eulThres = 6;   %minimum difference in Euler angle to compensate for jumps


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
time = [36,60];


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


%% Select suitable time frames
% Plot position data to search for time where x and y are constant
figure('Name','OptiTrack position data');
hold on;
plot(optitrackStampTime,optitrackPos(1,:),'-o');
plot(optitrackStampTime,optitrackPos(2,:),'-o');
plot(optitrackStampTime,optitrackPos(3,:),'-o');

% Enter otStart and otEnd here by:
% - Visually determining the time interval
% - Finding the samples corresponding to the beginning and end of the
%   interval using:
%   "[~,otStart] = min(abs(optitrackStampTime-startTime))
%   [~,otEnd] = min(abs(optitrackStampTime-endTime))"
keyboard;

% Select proper OptiTrack data
optitrackStampTime = optitrackStampTime(otStart:otEnd);
optitrackPos = optitrackPos(:,otStart:otEnd);
optitrackOrientQuat = optitrackOrientQuat(:,otStart:otEnd);

% Search for closest value for otStart and otEnd time in
% ardroneNavdataStampTime
[~,arStart] = min(abs(ardroneNavdataStampTime-optitrackStampTime(1)));
[~,arEnd] = min(abs(ardroneNavdataStampTime-optitrackStampTime(end)));

% Select desired Motor PWM data
ardroneNavdataStampTime = ardroneNavdataStampTime(arStart:arEnd);
ardroneNavdataMotor = ardroneNavdataMotor(:,arStart:arEnd);


%% Convert OptiTrack quaternions to ZYX Euler angles
% Convert quaternions to Euler angles
optitrackOrientQuat = optitrackOrientQuat';
orient = zeros(size(optitrackOrientQuat,1),3);
for i = 1:size(optitrackOrientQuat,1)
    orient(i,:) = quat2eul(optitrackOrientQuat(i,:));
end
orient = orient';

% Remove jumps of 2*pi in angle data and ensure the angles are centered
% around 0 rad
optitrackOrient = unwrap(orient,eulThres,2);
for i = 1:3
    if mean(optitrackOrient(i,:)) > eulThres/2
        optitrackOrient(i,:) = optitrackOrient(i,:) - pi;
    elseif mean(optitrackOrient(i,:)) < -eulThres/2
        optitrackOrient(i,:) = optitrackOrient(i,:) + pi;
    end
end

figure('Name','OptiTrack orientation data');
subplot(3,1,1);
plot(optitrackStampTime,optitrackOrient(1,:),'-o');
title('\phi');
subplot(3,1,2);
plot(optitrackStampTime,optitrackOrient(2,:),'-o');
title('\theta');
subplot(3,1,3);
plot(optitrackStampTime,optitrackOrient(3,:),'-o');
title('\psi');


%% Interpolate data
% Sample time
expData.sampleTimeLow = 0.04;
expData.sampleTimeHigh = 0.01;

% OptiTrack data
data.time = optitrackStampTime;
data.value = [optitrackPos;optitrackOrient];
tmp = interpolate(expData.sampleTimeLow,data);
expData.state.lowFreq.otTime = tmp.time;
expData.state.lowFreq.otPos = tmp.value(1:3,:);
expData.state.lowFreq.otOrient = tmp.value(4:6,:);
tmp = interpolate(expData.sampleTimeHigh,data);
expData.state.highFreq.otTime = tmp.time;
expData.state.highFreq.otPos = tmp.value(1:3,:);
expData.state.highFreq.otOrient = tmp.value(4:6,:);

% AR.Drone 2.0 motor PWM data
data.time = ardroneNavdataStampTime;
data.value = ardroneNavdataMotor;
tmp = interpolate(expData.state.lowFreq.otTime,data);
expData.input.lowFreq.time = tmp.time;
expData.input.lowFreq.motor = tmp.value;


%% Ensure that the data sampled at a high frequency has enough samples
%  before the first sample of the data sampled at a low frequency
expData.state.lowFreq.otTime = expData.state.lowFreq.otTime(2:end);
expData.state.lowFreq.otPos = expData.state.lowFreq.otPos(:,2:end);
expData.state.lowFreq.otOrient = expData.state.lowFreq.otOrient(:,2:end);


%% Ensure data is consistent: same start and end time + start at zero
% Start time
if expData.input.lowFreq.time(1) < expData.state.lowFreq.otTime(1)
    i = 2;
    while expData.input.lowFreq.time(i) < expData.state.lowFreq.otTime(1)
        i = i + 1;
    end
    expData.input.lowFreq.time = expData.input.lowFreq.time(i:end);
    expData.input.lowFreq.motor = expData.input.lowFreq.motor(:,i:end);
elseif expData.input.lowFreq.time(1) > expData.state.lowFreq.otTime(1)
    i = 2;
    while expData.input.lowFreq.time(1) > expData.state.lowFreq.otTime(i)
        i = i + 1;
    end
    expData.state.lowFreq.otTime = expData.state.lowFreq.otTime(i:end);
    expData.state.lowFreq.otPos = expData.state.lowFreq.otPos(:,i:end);
    expData.state.lowFreq.otOrient = ...
        expData.state.lowFreq.otOrient(:,i:end);
end

% End time
if expData.input.lowFreq.time(end) > expData.state.lowFreq.otTime(end)
    i = length(expData.input.lowFreq.time);
    while expData.input.lowFreq.time(i) > expData.state.lowFreq.otTime(end)
        i = i - 1;
    end
    expData.input.lowFreq.time = expData.input.lowFreq.time(1:i);
    expData.input.lowFreq.motor = expData.input.lowFreq.motor(:,1:i);
elseif expData.input.lowFreq.time(end) < expData.state.lowFreq.otTime(end)
    i = length(expData.state.lowFreq.otTime);
    while expData.input.lowFreq.time(end) < expData.state.lowFreq.otTime(i)
        i = i - 1;
    end
    expData.state.lowFreq.otTime = expData.state.lowFreq.otTime(1:i);
    expData.state.lowFreq.otPos = expData.state.lowFreq.otPos(:,1:i);
    expData.state.lowFreq.otOrient = expData.state.lowFreq.otOrient(:,1:i);
end

expData.input.lowFreq.time	  = expData.input.lowFreq.time - ...
                                expData.input.lowFreq.time(1);
expData.state.lowFreq.otTime  = expData.state.lowFreq.otTime - ...
                                expData.state.lowFreq.otTime(1);
expData.state.highFreq.otTime = expData.state.highFreq.otTime - ...
                                expData.state.highFreq.otTime(1);


%% Save gazSim data
filename = sprintf('bagdata_%s', datestr(now,'dd-mm-yyyy_HH-MM'));
save(filename, 'expData');
