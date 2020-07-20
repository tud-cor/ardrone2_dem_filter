function [topicsOut] = storeBagdata(bag, topics, time)
% STOREBAGDATA Store data of a rosbag in Matlab matrices.
%
%   Author: Dennis Benders
%   Last edited: 08.05.2020
%
%   Input:	bagname:    string with name of bag (excluding ".bag")
%           topics:     struct with selection of the data you want to be
%                       stored
%           time:       double array: 1st index is starting time, 2nd index
%                       is stoptime, until maximum of the total recorded
%                       time is hit
%
%   Output: topicsOut: struct containing all (row) arrays as specified in
%                       input
%
%   For usage, see getSimData.m.

%--------------------------------------------------------------------------
% Check input arguments
if nargin < 3
    error("STOREBAGDATA: please specify all 3 input arguments.");
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Check timing data
if time(1) > time(2)
    error("STOREBAGDATA: end time must be larger than start time.");
elseif bag.EndTime < bag.StartTime + time(1)
    error("STOREBAGDATA: start time out of bounds.");
elseif bag.EndTime < bag.StartTime + time(2)
    warning("STOREBAGDATA: end time out of bounds.\n%s", ...
        "Your data will be cut off at the end of the available time.");
end

% Calculate total recording time to be stored in arrays
endTime = min(bag.EndTime, bag.StartTime + time(2));
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Retreive information from cmd_vel if desired
if topics.cmdVel
    % Get bag data
    bagCmdVel = select(bag, ...
        'Time', [bag.StartTime + time(1), endTime], ...
        'Topic', '/cmd_vel');
    msgsCmdVel = readMessages(bagCmdVel, 'DataFormat', 'Struct');
    lenCmdVel = length(msgsCmdVel);

    % Store data of cmd_vel in arrays
    topicsOut.cmdVel.time = zeros(1, lenCmdVel);
    topicsOut.cmdVel.lin = zeros(3, lenCmdVel);
    topicsOut.cmdVel.ang = zeros(3, lenCmdVel);

    for i = 1:lenCmdVel
        topicsOut.cmdVel.time(i) = bagCmdVel.MessageList.Time(i);
        topicsOut.cmdVel.lin(1,i) = msgsCmdVel{i}.Linear.X;
        topicsOut.cmdVel.lin(2,i) = msgsCmdVel{i}.Linear.Y;
        topicsOut.cmdVel.lin(3,i) = msgsCmdVel{i}.Linear.Z;
        topicsOut.cmdVel.ang(1,i) = msgsCmdVel{i}.Angular.X;
        topicsOut.cmdVel.ang(2,i) = msgsCmdVel{i}.Angular.Y;
        topicsOut.cmdVel.ang(3,i) = msgsCmdVel{i}.Angular.Z;
    end

    % Remove unncessary data for later on in this function to save memory
    clear bagCmdVel msgsCmdVel;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Retreive information from ardrone2_dem/model_input if desired
if topics.modelInput
    % Get bag data
    bagModelInput = select(bag, ...
        'Time', [bag.StartTime + time(1), endTime], ...
        'Topic', '/ardrone2_dem/model_input');
    msgsModelInput = readMessages(bagModelInput, 'DataFormat', 'Struct');
    lenModelInput = length(msgsModelInput);

    % Store data of model_input in arrays
    topicsOut.modelInput.time = zeros(1, lenModelInput);
    topicsOut.modelInput.force = zeros(3, lenModelInput);
    topicsOut.modelInput.torque = zeros(3, lenModelInput);

    for i = 1:lenModelInput
        topicsOut.modelInput.stampTime(i) = ...
            msgsModelInput{i}.Header.Stamp.Sec + ...
            msgsModelInput{i}.Header.Stamp.Nsec/1000000000;
        topicsOut.modelInput.recordTime(i) = ...
            bagModelInput.MessageList.Time(i);
        topicsOut.modelInput.force(1,i) = msgsModelInput{i}.Wrench.Force.X;
        topicsOut.modelInput.force(2,i) = msgsModelInput{i}.Wrench.Force.Y;
        topicsOut.modelInput.force(3,i) = msgsModelInput{i}.Wrench.Force.Z;
        topicsOut.modelInput.torque(1,i) = ...
            msgsModelInput{i}.Wrench.Torque.X;
        topicsOut.modelInput.torque(2,i) = ...
            msgsModelInput{i}.Wrench.Torque.Y;
        topicsOut.modelInput.torque(3,i) = ...
            msgsModelInput{i}.Wrench.Torque.Z;
    end

    % Remove unncessary data for later on in this function to save memory
    clear bagModelInput msgsModelInput;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Retreive information from gazebo/model_states if desired
if topics.gazeboModelStates
    % Get bag data
    bagGazeboModelStates = select(bag, ...
        'Time', [bag.StartTime + time(1), endTime], ...
        'Topic', '/gazebo/model_states');
    msgsGazeboModelStates = readMessages(bagGazeboModelStates, ...
                                         'DataFormat', 'Struct');
    lenGazeboModelStates = length(msgsGazeboModelStates);

    % Store data of model_states in arrays
    topicsOut.gazeboModelStates.time = zeros(1, lenGazeboModelStates);
    topicsOut.gazeboModelStates.pos = zeros(3, lenGazeboModelStates);
    topicsOut.gazeboModelStates.orient = zeros(4, lenGazeboModelStates);
    topicsOut.gazeboModelStates.vlin = zeros(3, lenGazeboModelStates);
    topicsOut.gazeboModelStates.vang = zeros(3, lenGazeboModelStates);

    for i = 1:lenGazeboModelStates
        topicsOut.gazeboModelStates.time(i) = ...
            bagGazeboModelStates.MessageList.Time(i);
        topicsOut.gazeboModelStates.pos(1,i) = ...
            msgsGazeboModelStates{i}.Pose(2).Position.X;
        topicsOut.gazeboModelStates.pos(2,i) = ...
            msgsGazeboModelStates{i}.Pose(2).Position.Y;
        topicsOut.gazeboModelStates.pos(3,i) = ...
            msgsGazeboModelStates{i}.Pose(2).Position.Z;
        topicsOut.gazeboModelStates.orient(1,i) = ...
            msgsGazeboModelStates{i}.Pose(2).Orientation.X;
        topicsOut.gazeboModelStates.orient(2,i) = ...
            msgsGazeboModelStates{i}.Pose(2).Orientation.Y;
        topicsOut.gazeboModelStates.orient(3,i) = ...
            msgsGazeboModelStates{i}.Pose(2).Orientation.Z;
        topicsOut.gazeboModelStates.orient(4,i) = ...
            msgsGazeboModelStates{i}.Pose(2).Orientation.W;
        topicsOut.gazeboModelStates.vLin(1,i) = ...
            msgsGazeboModelStates{i}.Twist(2).Linear.X;
        topicsOut.gazeboModelStates.vLin(2,i) = ...
            msgsGazeboModelStates{i}.Twist(2).Linear.Y;
        topicsOut.gazeboModelStates.vLin(3,i) = ...
            msgsGazeboModelStates{i}.Twist(2).Linear.Z;
        topicsOut.gazeboModelStates.vAng(1,i) = ...
            msgsGazeboModelStates{i}.Twist(2).Angular.X;
        topicsOut.gazeboModelStates.vAng(2,i) = ...
            msgsGazeboModelStates{i}.Twist(2).Angular.Y;
        topicsOut.gazeboModelStates.vAng(3,i) = ...
            msgsGazeboModelStates{i}.Twist(2).Angular.Z;
    end

    % Remove unncessary data for later on in this function to save memory
    clear bagGazeboModelStates msgsGazeboModelStates;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Retreive information from ardrone/odometry if desired
if topics.ardroneOdom
    % Get bag data
    bagArdroneOdom = select(bag, ...
        'Time', [bag.StartTime + time(1), endTime], ...
        'Topic', '/ardrone/odometry');
    msgsArdroneOdom = readMessages(bagArdroneOdom, 'DataFormat', 'Struct');
    lenArdroneOdom = length(msgsArdroneOdom);

    % Store data of model_states in arrays
    topicsOut.ardroneOdom.time = zeros(1, lenArdroneOdom);
    topicsOut.ardroneOdom.pos = zeros(3, lenArdroneOdom);
    topicsOut.ardroneOdom.orient = zeros(4, lenArdroneOdom);
    topicsOut.ardroneOdom.vlin = zeros(3, lenArdroneOdom);
    topicsOut.ardroneOdom.vang = zeros(3, lenArdroneOdom);

    for i = 1:lenArdroneOdom
        topicsOut.ardroneOdom.stampTime(i) = ...
            msgsArdroneOdom{i}.Header.Stamp.Sec + ...
            msgsArdroneOdom{i}.Header.Stamp.Nsec/1000000000;
        topicsOut.ardroneOdom.recordTime(i) = ...
            bagArdroneOdom.MessageList.Time(i);
        topicsOut.ardroneOdom.pos(1,i) = ...
            msgsArdroneOdom{i}.Pose.Pose.Position.X;
        topicsOut.ardroneOdom.pos(2,i) = ...
            msgsArdroneOdom{i}.Pose.Pose.Position.Y;
        topicsOut.ardroneOdom.pos(3,i) = ...
            msgsArdroneOdom{i}.Pose.Pose.Position.Z;
        topicsOut.ardroneOdom.orient(1,i) = ...
            msgsArdroneOdom{i}.Pose.Pose.Orientation.X;
        topicsOut.ardroneOdom.orient(2,i) = ...
            msgsArdroneOdom{i}.Pose.Pose.Orientation.Y;
        topicsOut.ardroneOdom.orient(3,i) = ...
            msgsArdroneOdom{i}.Pose.Pose.Orientation.Z;
        topicsOut.ardroneOdom.orient(4,i) = ...
            msgsArdroneOdom{i}.Pose.Pose.Orientation.W;
        topicsOut.ardroneOdom.vLin(1,i) = ...
            msgsArdroneOdom{i}.Twist.Twist.Linear.X;
        topicsOut.ardroneOdom.vLin(2,i) = ...
            msgsArdroneOdom{i}.Twist.Twist.Linear.Y;
        topicsOut.ardroneOdom.vLin(3,i) = ...
            msgsArdroneOdom{i}.Twist.Twist.Linear.Z;
        topicsOut.ardroneOdom.vAng(1,i) = ...
            msgsArdroneOdom{i}.Twist.Twist.Angular.X;
        topicsOut.ardroneOdom.vAng(2,i) = ...
            msgsArdroneOdom{i}.Twist.Twist.Angular.Y;
        topicsOut.ardroneOdom.vAng(3,i) = ...
            msgsArdroneOdom{i}.Twist.Twist.Angular.Z;
    end

    % Remove unncessary data for later on in this function to save memory
    clear bagArdroneOdom msgsArdroneOdom;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Retreive information from ardrone/motor_speed if desired
if topics.rotorsMotorSpeed
    % Get bag data
    bagMotorSpeed = select(bag, ...
        'Time', [bag.StartTime + time(1), endTime], ...
        'Topic', '/ardrone/motor_speed');
    msgsMotorSpeed = readMessages(bagMotorSpeed, 'DataFormat', 'Struct');
    lenMotorSpeed = length(msgsMotorSpeed);

    % Store data of ardrone/motor_speed in arrays
    topicsOut.motorSpeed.time = zeros(1, lenMotorSpeed);
    topicsOut.motorSpeed.ang_vel = zeros(4, lenMotorSpeed);

    for i = 1:lenMotorSpeed
        topicsOut.motorSpeed.time(i) = bagMotorSpeed.MessageList.Time(i);
        topicsOut.motorSpeed.angVel(1,i) = ...
            msgsMotorSpeed{i}.AngularVelocities(1);
        topicsOut.motorSpeed.angVel(2,i) = ...
            msgsMotorSpeed{i}.AngularVelocities(2);
        topicsOut.motorSpeed.angVel(3,i) = ...
            msgsMotorSpeed{i}.AngularVelocities(3);
        topicsOut.motorSpeed.angVel(4,i) = ...
            msgsMotorSpeed{i}.AngularVelocities(4);
    end

    % Remove unncessary data for later on in this function to save memory
    clear bagMotorSpeed msgsMotorSpeed;
end
%--------------------------------------------------------------------------