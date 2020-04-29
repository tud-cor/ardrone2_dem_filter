function [topicsOut] = storeBagdata(bag, topics, time)
% STOREBAGDATA Store data of a rosbag in Matlab matrices
%
%   Author: Dennis Benders, TU Delft
%   Last edited: 29.04.2020
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
%   For usage, see get_rosbag_data.m.

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
        topicsOut.modelInput.time(i) = msgsModelInput{i}.Header.Stamp.Sec + msgsModelInput{i}.Header.Stamp.Nsec/1000000000;
        topicsOut.modelInput.force(1,i) = msgsModelInput{i}.Wrench.Force.X;
        topicsOut.modelInput.force(2,i) = msgsModelInput{i}.Wrench.Force.Y;
        topicsOut.modelInput.force(3,i) = msgsModelInput{i}.Wrench.Force.Z;
        topicsOut.modelInput.torque(1,i) = msgsModelInput{i}.Wrench.Torque.X;
        topicsOut.modelInput.torque(2,i) = msgsModelInput{i}.Wrench.Torque.Y;
        topicsOut.modelInput.torque(3,i) = msgsModelInput{i}.Wrench.Torque.Z;
    end
    
    % Remove unncessary data for later on in this function to save memory
    clear bagModelInput msgsModelInput;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Retreive information from gazebo/model_states if desired
if topics.modelStates
    % Get bag data
    bagModelStates = select(bag, ...
        'Time', [bag.StartTime + time(1), endTime], ...
        'Topic', '/gazebo/model_states');
    msgsModelStates = readMessages(bagModelStates, 'DataFormat', 'Struct');
    lenModelStates = length(msgsModelStates);
    
    % Store data of model_states in arrays
    topicsOut.modelStates.time = zeros(1, lenModelStates);
    topicsOut.modelStates.pos = zeros(3, lenModelStates);
    topicsOut.modelStates.orient = zeros(4, lenModelStates);
    topicsOut.modelStates.vlin = zeros(3, lenModelStates);
    topicsOut.modelStates.vang = zeros(3, lenModelStates);

    for i = 1:lenModelStates
        topicsOut.modelStates.time(i) = bagModelStates.MessageList.Time(i);
        topicsOut.modelStates.pos(1,i) = msgsModelStates{i}.Pose(2).Position.X;
        topicsOut.modelStates.pos(2,i) = msgsModelStates{i}.Pose(2).Position.Y;
        topicsOut.modelStates.pos(3,i) = msgsModelStates{i}.Pose(2).Position.Z;
        topicsOut.modelStates.orient(1,i) = msgsModelStates{i}.Pose(2).Orientation.X;
        topicsOut.modelStates.orient(2,i) = msgsModelStates{i}.Pose(2).Orientation.Y;
        topicsOut.modelStates.orient(3,i) = msgsModelStates{i}.Pose(2).Orientation.Z;
        topicsOut.modelStates.orient(4,i) = msgsModelStates{i}.Pose(2).Orientation.W;
        topicsOut.modelStates.vLin(1,i) = msgsModelStates{i}.Twist(2).Linear.X;
        topicsOut.modelStates.vLin(2,i) = msgsModelStates{i}.Twist(2).Linear.Y;
        topicsOut.modelStates.vLin(3,i) = msgsModelStates{i}.Twist(2).Linear.Z;
        topicsOut.modelStates.vAng(1,i) = msgsModelStates{i}.Twist(2).Angular.X;
        topicsOut.modelStates.vAng(2,i) = msgsModelStates{i}.Twist(2).Angular.Y;
        topicsOut.modelStates.vAng(3,i) = msgsModelStates{i}.Twist(2).Angular.Z;
    end
    
    % Remove unncessary data for later on in this function to save memory
    clear bagModelStates msgsModelStates;
end
%--------------------------------------------------------------------------