%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get recorded experiment data
%
% Script to read out a bag file with recorded flight experiment data,
% process the data according to coordinate frame and angle conventions and
% save original as well as interpolated data (120 Hz) in a .mat file.
% 
% Author:        Dennis Benders, TU Delft, CoR
% Last modified: 21.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear;
close all;
clc;


%% Adjustable parameters
% Name of rosbag in ./ros
bagname = 'ardrone2_exp_2020-10-29_25_batP1.bag';

% Time interval with respect to start of rosbag recording
time = [0,80]; %exp10-29_25 - wind mode 2

% Topic selection
topics.cmdVel = 0;
topics.modelInput = 0;
topics.gazeboModelStates = 0;
topics.rotorsMotorSpeed = 0;
topics.optitrack = 1;
topics.ardroneImu = 1;
topics.ardroneNav = 1;
topics.ardroneOdom = 1;

% Also store data interpolated at a high frequency
highFreq = 0;


%% Constant parameters
deg2rad = 2*pi/180; %conversion factor from degrees to radians
eulThres = 6;       %minimum difference in Euler angle to compensate for jumps
floatTol = 1e-6;    %number with a smaller accuracy are treated as 0
fs = 120;           %maximum sample frequency in Hz (limited by OptiTrack)
g = 9.81;           %gravitational constant
sampleThresOt = 10; %minimum amount of samples in the beginning and end of
                    %Optitrack data that should be ignored in order to
                    %successfully interpolate the data


%% Get bag data
% Retrieve bag file
cd ~/.ros;
bag = rosbag(bagname);
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

% topicsOut = storeBagdata(bag,topics,time);
load getExpData10_29_25.mat;

if topics.cmdVel
    cmdVelTime = topicsOut.cmdVel.time;
    cmdVelLin  = topicsOut.cmdVel.lin;
    cmdVelAng  = topicsOut.cmdVel.ang;
end

if topics.optitrack
    optitrackStampTime  = topicsOut.optitrack.stampTime;
    optitrackRecordTime = topicsOut.optitrack.recordTime;
    optitrackPos        = topicsOut.optitrack.pos;
    optitrackOrientQuat = topicsOut.optitrack.orient;
end

if topics.ardroneImu
    ardroneImuStampTime  = topicsOut.ardroneImu.stampTime;
    ardroneImuALin       = topicsOut.ardroneImu.aLin;
    ardroneImuOrientQuat = topicsOut.ardroneImu.orient;
    ardroneImuVAng       = topicsOut.ardroneImu.vAng;
end

if topics.ardroneNav
    ardroneNavStampTime  = topicsOut.ardroneNav.stampTime;
    ardroneNavRecordTime = topicsOut.ardroneNav.recordTime;
    ardroneNavMotor      = topicsOut.ardroneNav.motor;
    ardroneNavAltd       = topicsOut.ardroneNav.altd;
    ardroneNavVLin       = topicsOut.ardroneNav.vLin;
    ardroneNavALin       = topicsOut.ardroneNav.aLin;
    ardroneNavRot        = topicsOut.ardroneNav.rot;
end

if topics.ardroneOdom
    ardroneOdomStampTime  = topicsOut.ardroneOdom.stampTime;
    ardroneOdomRecordTime = topicsOut.ardroneOdom.recordTime;
    ardroneOdomPos        = topicsOut.ardroneOdom.pos;
    ardroneOdomVLin       = topicsOut.ardroneOdom.vLin;
    ardroneOdomOrientQuat = topicsOut.ardroneOdom.orient;
    ardroneOdomVAng       = topicsOut.ardroneOdom.vAng;
end


%% Ensure that all linear positions start at 0 and are given in [m]

% TODO Check convention of odometry position!

ardroneNavAltd = ardroneNavAltd/1000;

% Remove offsets
% TODO only do this if position data of OptiTrack needs to be aligned with
% that coming from AR.Drone 2.0
% optitrackPos = optitrackPos - optitrackPos(:,1);
% ardroneNavAltd = ardroneNavAltd - ardroneNavAltd(1);
% ardroneOdomPos = ardroneOdomPos - ardroneOdomPos(:,1);


%% Ensure that all linear velocities are given in [m/s]
ardroneNavVLin = ardroneNavVLin/1000;

% No adjustments to odometry


%% Ensure that all linear accelerations are given in [m/s^2]
% No adjustments to IMU (take care: z is starting at ~9.81 m/s^2)

% (take care: z is starting at ~9.81 m/s^2)
ardroneNavALin = ardroneNavALin*g;


%% Ensure that all orientations are given in ZYX Euler angles, start at 0
%  and are given in [rad]

% TODO Convert all angles from AR.Drone 2.0 to inertial frame!

% Convert quaternion from ROS convention (x,y,z,w)
%                    to MATLAB convention (w,x,y,z)
optitrackOrientQuat      = [optitrackOrientQuat(4,:);...
                            optitrackOrientQuat(1:3,:)];
if topics.ardroneImu
    ardroneImuOrientQuat = [ardroneImuOrientQuat(4,:);...
                            ardroneImuOrientQuat(1:3,:)];
end
ardroneOdomOrientQuat    = [ardroneOdomOrientQuat(4,:);...
                            ardroneOdomOrientQuat(1:3,:)];

% Convert quaternions to ZYX Euler angles: [Z;Y;X] ([psi;theta;phi])
optitrackOrient      = quat2eul(optitrackOrientQuat','ZYX')';
if topics.ardroneImu
    ardroneImuOrient = quat2eul(ardroneImuOrientQuat','ZYX')';
end
ardroneOdomOrient    = quat2eul(ardroneOdomOrientQuat','ZYX')';


% Convert navdata to radians and to [Z;Y;X] ([psi;theta;phi]) convention
ardroneNavOrient = ardroneNavRot*deg2rad;
ardroneNavOrient = [ardroneNavOrient(3,:);ardroneNavOrient(2,:);...
                    ardroneNavOrient(1,:)];

% Remove offsets
% TODO only do this if orientation data of OptiTrack needs to be aligned
% with that coming from AR.Drone 2.0 -> using rotation matrices, not the
% method implemented below!
% optitrackOrient   = optitrackOrient - optitrackOrient(:,1);
% if topics.ardroneImu
%     ardroneImuOrient  = ardroneImuOrient - ardroneImuOrient(:,1);
% end
% ardroneOdomOrient = ardroneOdomOrient - ardroneOdomOrient(:,1);
% ardroneNavOrient  = ardroneNavOrient - ardroneNavOrient(:,1);


%% Ensure all angular velocities are given in [rad/s]
%  with convention [phiDot;thetaDot;psiDot]
% No adjustments to IMU

% Odometry is 0!


%% Save original data in expData struct with aligned times starting at 0
% Save data
expData.origData.otTime   = optitrackStampTime;
expData.origData.otPos    = optitrackPos;
expData.origData.otOrient = optitrackOrient;

if topics.ardroneImu
    expData.origData.imuTime   = ardroneImuStampTime;
    expData.origData.imuALin   = ardroneImuALin;
    expData.origData.imuOrient = ardroneImuOrient;
    expData.origData.imuVAng   = ardroneImuVAng;
end

expData.origData.navTime   = ardroneNavStampTime;
expData.origData.navMotor  = ardroneNavMotor;
expData.origData.navAltd   = ardroneNavAltd;
expData.origData.navVLin   = ardroneNavVLin;
expData.origData.navALin   = ardroneNavALin;
expData.origData.navOrient = ardroneNavOrient;

expData.origData.odomTime   = ardroneOdomStampTime;
expData.origData.odomPos    = ardroneOdomPos;
expData.origData.odomVLin   = ardroneOdomVLin;
expData.origData.odomOrient = ardroneOdomOrient;
expData.origData.odomVAng   = ardroneOdomVAng;


if topics.ardroneImu
    % Align time and start at 0
    [~,idx] = min([optitrackStampTime(1);ardroneImuStampTime(1);...
                   ardroneNavStampTime(1);ardroneOdomStampTime(1)]);
    if idx == 1
        expData.origData.imuTime  = expData.origData.imuTime  - ...
                                    optitrackStampTime(1);
        expData.origData.navTime  = expData.origData.navTime  - ...
                                    optitrackStampTime(1);
        expData.origData.odomTime = expData.origData.odomTime - ...
                                    optitrackStampTime(1);
        expData.origData.otTime   = expData.origData.otTime   - ...
                                    optitrackStampTime(1);
    elseif idx == 2
        expData.origData.otTime   = expData.origData.otTime   - ...
                                    ardroneImuStampTime(1);
        expData.origData.navTime  = expData.origData.navTime  - ...
                                    ardroneImuStampTime(1);
        expData.origData.odomTime = expData.origData.odomTime - ...
                                    ardroneImuStampTime(1);
        expData.origData.imuTime  = expData.origData.imuTime  - ...
                                    ardroneImuStampTime(1);
    elseif idx == 3
        expData.origData.otTime   = expData.origData.otTime   - ...
                                    ardroneNavStampTime(1);
        expData.origData.imuTime  = expData.origData.imuTime  - ...
                                    ardroneNavStampTime(1);
        expData.origData.odomTime = expData.origData.odomTime - ...
                                    ardroneNavStampTime(1);
        expData.origData.navTime  = expData.origData.navTime  - ...
                                    ardroneNavStampTime(1);
    elseif idx == 4
        expData.origData.otTime   = expData.origData.otTime   - ...
                                    ardroneOdomStampTime(1);
        expData.origData.imuTime  = expData.origData.imuTime  - ...
                                    ardroneOdomStampTime(1);
        expData.origData.navTime  = expData.origData.navTime  - ...
                                    ardroneOdomStampTime(1);
        expData.origData.odomTime = expData.origData.odomTime - ...
                                    ardroneOdomStampTime(1);
    end
else
    % Align time and start at 0
    [~,idx] = min([optitrackStampTime(1);ardroneNavStampTime(1);...
                   ardroneOdomStampTime(1)]);
    if idx == 1
        ardroneNavStampTime  = ardroneNavStampTime  - ...
                               optitrackStampTime(1);
        ardroneOdomStampTime = ardroneOdomStampTime - ...
                               optitrackStampTime(1);
        optitrackStampTime   = optitrackStampTime   - ...
                               optitrackStampTime(1);
    elseif idx == 2
        optitrackStampTime   = optitrackStampTime   - ...
                               ardroneNavStampTime(1);
        ardroneOdomStampTime = ardroneOdomStampTime - ...
                               ardroneNavStampTime(1);
        ardroneNavStampTime  = ardroneNavStampTime  - ...
                               ardroneNavStampTime(1);
    elseif idx == 3
        optitrackStampTime   = optitrackStampTime   - ...
                               ardroneOdomStampTime(1);
        ardroneNavStampTime  = ardroneNavStampTime  - ...
                               ardroneOdomStampTime(1);
        ardroneOdomStampTime = ardroneOdomStampTime - ...
                               ardroneOdomStampTime(1);
    end
end


%% Select suitable time frames
% Plot position data to search for time where x and y are constant
optitrackPlotTime = optitrackStampTime - optitrackStampTime(1);
figure('Name','OptiTrack position data');
hold on;
plot(optitrackPlotTime,optitrackPos(1,:),'-o');
plot(optitrackPlotTime,optitrackPos(2,:),'-o');
plot(optitrackPlotTime,optitrackPos(3,:),'-o');
keyboard;

% Select data samples to use
prompt      = {'Enter time of 1st data sample:',...
               'Enter time of last data sample:'};
dlgtitle    = 'Data selection';
dims        = [1 35];
definput    = {num2str(optitrackPlotTime(sampleThresOt)),...
               num2str(optitrackPlotTime(end-sampleThresOt))};
answer      = inputdlg(prompt,dlgtitle,dims,definput);
startTime   = round(str2double(answer{1})) + optitrackStampTime(1);
endTime     = round(str2double(answer{2})) + optitrackStampTime(1);
[~,otStart] = min(abs(optitrackStampTime-startTime));
[~,otEnd]   = min(abs(optitrackStampTime-endTime));
if otStart < sampleThresOt
    error(['Please give a higher index of the 1st sample to ensure '...
           'data consistency after interpolation']);
end
if otEnd > length(optitrackStampTime) - sampleThresOt
    error(['Please give a lower index of the last sample to ensure '...
           'data consistency after interpolation']);
end

% Select proper OptiTrack data
optitrackStampTime = optitrackStampTime(otStart:otEnd);
optitrackPos       = optitrackPos(:,otStart:otEnd);
optitrackOrient    = optitrackOrient(:,otStart:otEnd);

% Find AR.Drone 2.0 IMU data that fits within the Optitrack data
if topics.ardroneImu
    [imuStart,imuEnd]   = findStartEndIdx(ardroneImuStampTime,...
                                           optitrackStampTime,floatTol);
    ardroneImuStampTime = ardroneImuStampTime(imuStart:imuEnd);
    ardroneImuALin      = ardroneImuALin(:,imuStart:imuEnd);
    ardroneImuOrient    = ardroneImuOrient(:,imuStart:imuEnd);
    ardroneImuVAng      = ardroneImuVAng(:,imuStart:imuEnd);
end

% Find AR.Drone 2.0 navdata that fits within the Optitrack data
[navStart,navEnd]   = findStartEndIdx(ardroneNavStampTime,...
                                      optitrackStampTime,floatTol);
ardroneNavStampTime = ardroneNavStampTime(navStart:navEnd);
ardroneNavMotor     = ardroneNavMotor(:,navStart:navEnd);
ardroneNavAltd      = ardroneNavAltd(navStart:navEnd);
ardroneNavVLin      = ardroneNavVLin(:,navStart:navEnd);
ardroneNavALin      = ardroneNavALin(:,navStart:navEnd);
ardroneNavOrient    = ardroneNavOrient(:,navStart:navEnd);

% Find AR.Drone 2.0 odometry data that fits within the Optitrack data
[odomStart,odomEnd]  = findStartEndIdx(ardroneOdomStampTime,...
                                        optitrackStampTime,floatTol);
ardroneOdomStampTime = ardroneOdomStampTime(odomStart:odomEnd);
ardroneOdomPos       = ardroneOdomPos(:,odomStart:odomEnd);
ardroneOdomVLin      = ardroneOdomVLin(:,odomStart:odomEnd);
ardroneOdomOrient    = ardroneOdomOrient(:,odomStart:odomEnd);
ardroneOdomVAng      = ardroneOdomVAng(:,odomStart:odomEnd);

% Align time frames and start at 0
if topics.ardroneImu
    ardroneImuStampTime  = ardroneImuStampTime  - optitrackStampTime(1);
end
ardroneNavStampTime  = ardroneNavStampTime  - optitrackStampTime(1);
ardroneOdomStampTime = ardroneOdomStampTime - optitrackStampTime(1);
optitrackStampTime   = optitrackStampTime   - optitrackStampTime(1);


%% Interpolate data
% Sampling time
ts = 1/fs;
if highFreq
    expData.sampleTime         = 4*ts;
    expData.sampleTimeHighFreq = ts;
else
    expData.sampleTime = ts;
end


% OptiTrack data
data.time  = optitrackStampTime;
data.value = [optitrackPos;optitrackOrient];

tmp = interpolate(expData.sampleTime,data);
expData.output.time     = tmp.time;
expData.output.otPos    = tmp.value(1:3,:);
expData.output.otOrient = tmp.value(4:6,:);

if highFreq
    tmp = interpolate(expData.sampleTimeHighFreq,data);
    expData.output.highFreq.time     = tmp.time;
    expData.output.highFreq.otPos    = tmp.value(1:3,:);
    expData.output.highFreq.otOrient = tmp.value(4:6,:);
end


% AR.Drone 2.0 IMU data
if topics.ardroneImu
    data.time  = ardroneImuStampTime;
    data.value = [ardroneImuALin;ardroneImuOrient;ardroneImuVAng];

    tmp = interpolate(expData.output.time,data);
    expData.output.imuALin   = tmp.value(1:3,:);
    expData.output.imuOrient = tmp.value(4:6,:);
    expData.output.imuVAng   = tmp.value(7:9,:);
end


% AR.Drone 2.0 navdata
data.time  = ardroneNavStampTime;
data.value = [ardroneNavMotor;ardroneNavAltd;ardroneNavVLin;...
              ardroneNavALin;ardroneNavOrient];

tmp = interpolate(expData.output.time,data);
expData.input.navMotor   = tmp.value(1:4,:);
expData.output.navAltd   = tmp.value(5,:);
expData.output.navVLin   = tmp.value(6:8,:);
expData.output.navALin   = tmp.value(9:11,:);
expData.output.navOrient = tmp.value(12:14,:);


% AR.Drone 2.0 odometry
data.time  = ardroneOdomStampTime;
data.value = [ardroneOdomPos;ardroneOdomVLin;ardroneOdomOrient;...
              ardroneOdomVAng];

tmp = interpolate(expData.output.time,data);
expData.output.odomPos    = tmp.value(1:3,:);
expData.output.odomVLin   = tmp.value(4:6,:);
expData.output.odomOrient = tmp.value(7:9,:);
expData.output.odomVAng   = tmp.value(10:12,:);


%% Ensure that the data sampled at a higher frequency has enough samples
%  at the beginning and end of the data to construct the derivatives
if highFreq
    % Inputs
    expData.input.time     = expData.input.time(2:end-1);
    expData.input.navMotor = expData.input.navMotor(:,2:end-1);

    % Outputs
    expData.output.time     = expData.output.time(2:end-1);
    expData.output.otPos    = expData.output.otPos(:,2:end-1);
    expData.output.otOrient = expData.output.otOrient(:,2:end-1);

    if topics.ardroneImu
        expData.output.imuALin   = expData.output.imuALin(:,2:end-1);
        expData.output.imuOrient = expData.output.imuOrient(:,2:end-1);
        expData.output.imuVAng   = expData.output.imuVAng(:,2:end-1);
    end

    expData.output.navAltd   = expData.output.navAltd(2:end-1);
    expData.output.navVLin   = expData.output.navVLin(:,2:end-1);
    expData.output.navALin   = expData.output.navALin(:,2:end-1);
    expData.output.navOrient = expData.output.navOrient(:,2:end-1);

    expData.output.odomPos    = expData.output.odomPos(:,2:end-1);
    expData.output.odomVLin   = expData.output.odomVLin(:,2:end-1);
    expData.output.odomOrient = expData.output.odomOrient(:,2:end-1);
    expData.output.odomVAng   = expData.output.odomVAng(:,2:end-1);
end


%% Save expData data
filename = sprintf('bagdata_%s',datestr(now,'dd-mm-yyyy_HH-MM'));
save(filename,'expData');
