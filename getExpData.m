%% Initialization
clear;
close all;
clc;

% Parameters
timeAcc = 2e-3; %start and end time accuracy
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

keyboard;   %enter otStart and otEnd here

% Select proper OptiTrack data
optitrackStampTime = optitrackStampTime(otStart:otEnd);
optitrackPos = optitrackPos(:,otStart:otEnd);
optitrackOrientQuat = optitrackOrientQuat(:,otStart:otEnd);

% Search for otStart and otEnd time in ardroneNavdataStampTime
arStart = find(abs(ardroneNavdataStampTime-optitrackStampTime(1)) ...
               < timeAcc);
arEnd = find(abs(ardroneNavdataStampTime-optitrackStampTime(end)) ...
             < timeAcc);

% Select desired Motor PWM data
ardroneNavdataStampTime = ardroneNavdataStampTime(arStart:arEnd);
ardroneNavdataMotor = ardroneNavdataMotor(:,arStart:arEnd);


%% Convert OptiTrack quaternions to ZYX Euler angles
%TODO: maybe create a function with this functionality
% optitrackStampTime = optitrackStampTime - optitrackStampTime(1);
optitrackOrientQuat = optitrackOrientQuat';
orient = zeros(size(optitrackOrientQuat,1),3);
for i = 1:size(optitrackOrientQuat,1)
    orient(i,:) = quat2eul(optitrackOrientQuat(i,:));
end
orient = orient';

for i = 1:3
    % Store indices where jumps occur in array
    n = size(orient,2);
    jumps = zeros(1,n);
    for j = 2:n
        if orient(i,j) > 0 && orient(i,j-1) < 0 && orient(i,j) - ...
                orient(i,j-1) > eulThres
            jumps(j) = 1;
        elseif orient(i,j) < 0 && orient(i,j-1) > 0 && orient(i,j) - ...
                orient(i,j-1) < eulThres
            jumps(j) = -1;
        end
    end

    % Add +/-pi depending on first jump. For each jump, add +/-2pi, such
    % that every values becomes approximately 0
    jumpsDown = find(jumps == -1);
    jumpsUp = find(jumps == 1);
    nDown = length(jumpsDown);
    nUp = length(jumpsUp);
    if nDown > 1 && nUp > 1
        if jumpsDown(1) < jumpsUp(1)
            orient(i,:) = orient(i,:) - pi;
            for j = 1:nDown-1
                orient(i,jumpsDown(j):jumpsUp(j)-1) = ...
                    orient(i,jumpsDown(j):jumpsUp(j)-1) + 2*pi;
            end
            if nDown > nUp
                orient(i,jumpsDown(j):end) = orient(i,jumpsDown(j):end) + 2*pi;
            else
                orient(i,jumpsDown(end):jumpsUp(end)-1) = ...
                    orient(i,jumpsDown(end):jumpsUp(end)-1) + 2*pi;
            end
        else
            orient(i,:) = orient(i,:) + pi;
            for j = 1:nUp-1
                orient(i,jumpsUp(j):jumpsDown(j)-1) = ...
                    orient(i,jumpsUp(j):jumpsDown(j)-1) + 2*pi;
            end
            if nUp > nDown
                orient(i,jumpsUp(j):end) = orient(i,jumpsUp(j):end) + 2*pi;
            else
                orient(i,jumpsUp(end):jumpsDown(end)-1) = ...
                    orient(i,jumpsUp(end):jumpsDown(end)-1) + 2*pi;
            end
        end
    end

    optitrackOrient = orient;
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
expData.sampleTime = 1e-3;

% OptiTrack data
data.time = optitrackStampTime;
data.value = [optitrackPos;optitrackOrient];
tmp = interpolate(0.001, data);
expData.state.otTime = tmp.time;
expData.state.otPos = tmp.value(1:3,:);
expData.state.otOrient = tmp.value(4:6,:);

% AR.Drone 2.0 motor PWM data
data.time = ardroneNavdataStampTime;
data.value = ardroneNavdataMotor;
tmp = interpolate(expData.state.otTime, data);
expData.input.time = tmp.time;
expData.input.motor = tmp.value;


%% Ensure data is consistent: same start and end time + start at zero
% Start time
if expData.input.time(1) < expData.state.otTime(1)
    i = 2;
    while expData.input.time(i) < expData.state.otTime(1)
        i = i + 1;
    end
    expData.input.time = expData.input.time(i:end);
    expData.input.motor = expData.input.motor(:,i:end);
elseif expData.input.time(1) > expData.state.otTime(1)
    i = 2;
    while expData.input.time(1) > expData.state.otTime(i)
        i = i + 1;
    end
    expData.state.otTime = expData.state.otTime(i:end);
    expData.state.otPos = expData.state.otPos(:,i:end);
end

% End time
if expData.input.time(end) > expData.state.otTime(end)
    i = length(expData.input.time);
    while expData.input.time(i) > expData.state.otTime(end)
        i = i - 1;
    end
    expData.input.time = expData.input.time(1:i);
    expData.input.motor = expData.input.motor(:,1:i);
elseif expData.input.time(end) < expData.state.otTime(end)
    i = length(expData.state.otTime);
    while expData.input.time(end) < expData.state.otTime(i)
        i = i - 1;
    end
    expData.state.otTime = expData.state.otTime(1:i);
    expData.state.otPos = expData.state.otPos(:,1:i);
end

expData.input.time = expData.input.time - expData.input.time(1);
expData.state.otTime = expData.state.otTime - expData.state.otTime(1);


%% Save gazSim data
filename = sprintf('bagdata_%s', datestr(now,'dd-mm-yyyy_HH-MM'));
save(filename, 'expData');
