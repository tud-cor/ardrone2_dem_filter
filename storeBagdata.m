function [topics_out] = storeBagdata(bag, topics, time)
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
%   Output: topics_out: struct containing all (row) arrays as specified in
%                       input
%
%   For usage, see get_rosbag_data.m.

%--------------------------------------------------------------------------
%Check input arguments
if nargin < 3
    error("STORE_BAGDATA: please specify all 3 input arguments.");
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%Check timing data
if time(1) > time(2)
    error("STORE_BAGDATA: end time must be larger than start time.");
elseif bag.EndTime < bag.StartTime + time(1)
    error("STORE_BAGDATA: start time out of bounds.");
elseif bag.EndTime < bag.StartTime + time(2)
    warning("STORE_BAGDATA: end time out of bounds.\n%s", ...
        "Your data will be cut off at the end of the available time.");
end

%Calculate total recording time to be stored in arrays
end_time = min(bag.EndTime, bag.StartTime + time(2));

%Initialize array with starting times of the different messages
start_times = zeros(1,5);

%Initialize the indication whether a certain topic has to be stored
model_input = 0;
% cmd_vel = 0;
% joint_states = 0;
% model = 0;
% imu = 0;
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%Retreive information from ardrone2_dem/model_input if desired
if topics.model_input
    model_input = 1;
    
    %Get bag data
    bag_model_input = select(bag, ...
        'Time', [bag.StartTime + time(1), end_time], ...
        'Topic', '/ardrone2_dem/model_input');
    msgs_model_input = readMessages(bag_model_input, 'DataFormat', 'Struct');
    len_model_input = length(msgs_model_input);
    
    %Store data of model_input in arrays
    topics_out.model_input.time = zeros(1, len_model_input);
    topics_out.model_input.force = zeros(3, len_model_input);
    topics_out.model_input.torque = zeros(3, len_model_input);

    for i = 1:len_model_input
            topics_out.model_input.time(i) = msgs_model_input{i}.Header.Stamp.Sec + msgs_model_input{i}.Header.Stamp.Nsec/1000000000;
            topics_out.model_input.force(1,i) = msgs_model_input{i}.Wrench.Force.X;
            topics_out.model_input.force(2,i) = msgs_model_input{i}.Wrench.Force.Y;
            topics_out.model_input.force(3,i) = msgs_model_input{i}.Wrench.Force.Z;
            topics_out.model_input.torque(1,i) = msgs_model_input{i}.Wrench.Torque.X;
            topics_out.model_input.torque(2,i) = msgs_model_input{i}.Wrench.Torque.Y;
            topics_out.model_input.torque(3,i) = msgs_model_input{i}.Wrench.Torque.Z;
    end
    
    %Update start_times array
    start_times(1) = topics_out.model_input.time(1);
    
    %Remove unncessary data for later on in this function to save memory
    clear bag_model_input msgs_model_input;
end
%--------------------------------------------------------------------------