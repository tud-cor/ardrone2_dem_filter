function [intData] = interpolate(ref, data)
%INTERPOLATE Interpolates data argument depending on sampling time or
%reference signal given by ref
%   
%   Author: Dennis Benders, TU Delft
%   Last edited: 29.04.2020
%
%   Input:	ref:        either a sample time (single number) or a reference
%                       time signal
%           data:       struct with time and value sequence that needs to
%                       be interpolated
%
%   Output: intData:    struct with interpolated time and value sequence
%
%   For usage, see TODO
%   TODO: introduce a time input argument indicating the start and stop
%   time of the interpolated data (with warnings, errors, etc.)

%--------------------------------------------------------------------------
% Process input arguments
% Check input arguments
if nargin < 2
    error("INTERPOLATE: please specify both input arguments.");
end

% TODO: check time constraints at the beginning and end
%--------------------------------------------------------------------------

% Determine input lenghts
nRef = length(ref);
nData = length(data.time);

% Determine data sequence limits
startTime = data.time(1);
endTime = data.time(end);

% nRef = 0 indicates a sampling time, which is expanded from the start time
% until the end time of the data signal
% TODO: start at start time given by time array (input)
intData.time = ref;
if nRef == 1
    sampleTime = ref;
    nTime = ceil((endTime - startTime)/ref);
    intData.time = zeros(1,nTime);
    intData.time(1) = startTime;
    for i = 2:nTime
        intData.time(i) = intData.time(i-1) + sampleTime;
    end
end

% Ensure proper interpolation
% (only interpolate on times that are contained in data):
% 1. data.time(1) < intData.time(1)
% 2. data.time(end) > intData.time(end)
% 1.
if data.time(1) > intData.time(1)
    i = 2;
    while data.time(1) > intData.time(i)
        i = i + 1;
    end
    intData.time = intData.time(i:end);
end
% 2.
if data.time(end) < intData.time(end)
    i = length(intData.time) - 1;
    while data.time(end) < intData.time(i)
        i = i - 1;
    end
    intData.time = intData.time(1:i);
end

nRef = length(intData.time);
dataIdx = 1;
for refIdx = 1:nRef
    % Get next time index where data.time >= ref
    while data.time(dataIdx) < intData.time(refIdx)
        dataIdx = dataIdx + 1;
    end
    
    % If data.time == ref: no interpolation needed, just use that sample
    if abs(data.time(dataIdx) - intData.time(refIdx)) < 1e-10
        intData.value(:,refIdx) = data.value(:,dataIdx);
        
    % If data.time > ref: interpolate using data.value(dataIdx) and
    % data.value(dataIdx-1) depending on the time difference
    else
        intData.value(:,refIdx) = data.value(:,dataIdx-1) + ...
            (data.value(:,dataIdx) - data.value(:,dataIdx-1))/ ...
            (data.time(dataIdx) - data.time(dataIdx-1))* ...
            (intData.time(refIdx) - data.time(dataIdx-1));
    end
end