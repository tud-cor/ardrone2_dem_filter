%% Initialization
clear;
close all;
clc;

% Parameters
eulThres = 6;   %minimum difference in Euler angle to compensate for jumps


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
topics.ardroneNavdata = 1;
topics.ardroneOdom = 0;
topics.rotorsMotorSpeed = 0;

% Set time interval with respect to start of rosbag recording
time = [15,120];


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

% Select data samples to use
prompt = {'Enter index of 1st data sample:',...
          'Enter index of last data sample:'};
dlgtitle = 'Data selection';
dims = [1 35];
definput = {'1',num2str(length(optitrackStampTime))};
answer = inputdlg(prompt,dlgtitle,dims,definput);
otStart = round(str2double(answer{1}));
otEnd = round(str2double(answer{2}));

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
expData.sampleTime = 0.04;
expData.sampleTimeHighFreq = 0.01;

% OptiTrack data
data.time = optitrackStampTime;
data.value = [optitrackPos;optitrackOrient];
tmp = interpolate(expData.sampleTime,data);
expData.state.otTime = tmp.time;
expData.state.otPos = tmp.value(1:3,:);
expData.state.otOrient = tmp.value(4:6,:);
tmp = interpolate(expData.sampleTimeHighFreq,data);
expData.state.highFreq.otTime = tmp.time;
expData.state.highFreq.otPos = tmp.value(1:3,:);
expData.state.highFreq.otOrient = tmp.value(4:6,:);

% AR.Drone 2.0 motor PWM data
data.time = ardroneNavdataStampTime;
data.value = ardroneNavdataMotor;
tmp = interpolate(expData.state.otTime,data);
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
    expData.state.otOrient = expData.state.otOrient(:,i:end);
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
    expData.state.otOrient = expData.state.otOrient(:,1:i);
end

startTime = min([min(expData.input.time),min(expData.state.otTime)]);
expData.input.time	  = expData.input.time - startTime;
expData.state.otTime  = expData.state.otTime - startTime;
expData.state.highFreq.otTime = expData.state.highFreq.otTime - startTime;

%% Ensure that the data sampled at a higher frequency has enough samples
%  at the beginning and end of the data to construct the derivatives
expData.input.time    = expData.input.time(2:end-1);
expData.input.motor    = expData.input.motor(:,2:end-1);

expData.state.otTime  = expData.state.otTime(2:end-1);
expData.state.otPos  = expData.state.otPos(:,2:end-1);
expData.state.otOrient  = expData.state.otOrient(:,2:end-1);


%% Save expData data
filename = sprintf('bagdata_%s', datestr(now,'dd-mm-yyyy_HH-MM'));
save(filename, 'expData');
