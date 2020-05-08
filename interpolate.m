function [intData] = interpolate(ref, data, time)
% INTERPOLATE Linearly interpolate data argument based on single sampling
% time/sampling time array indicating the time stamps on which the data
% needs to be interpolated. Optionally, a time vector can be provided
% containing the interval on which the data will be interpolated (provided
% that this interval is covered by the data and reference sampling time
% array).
%   
%   Author: Dennis Benders
%   Last edited: 08.05.2020
%
%   Input:  ref:        either a sample time (single number) or a reference
%                       time signal (array)
%           data:       struct with time and value arrays that need to be
%                       interpolated
%           time (opt): offset interval with respect to data.time in which
%                       the signal needs to be interpolated
%
%   Output: intData:    struct with interpolated time and value arrays
%
%   For usage, see getSimData.m


%------------------------------ Parameters --------------------------------
% Minimum interpolation time
minIntTime = 1;     %s

% Time difference accuracy for interpolation
timeThres = 1e-10;  %s
%--------------------------------------------------------------------------


%------------------------ Process input arguments -------------------------
nRef = length(ref);

% Check input
if nargin < 2
    error('interpolate:TooFewArguments', ...
          'Please specify two or three input arguments.');
end

%------------------------
% Multiple ROS messages recorded with rosbag record may contain the same
% timestamp, while the simulated model can not handle different values at
% one moment in time.
% The following sequence occurs: t = [0.001, 0.002, 0.002, 0.004].
% This should be changed into:   t = [0.001, 0.002, 0.003, 0.004].
if nRef > 1
    ref = fixDups(ref);
end
data.time = fixDups(data.time);
%------------------------

if nargin == 3
    if time(1) < 0 || time(2) < 0
        error('interpolate:TimeNegative', ...
              'Please specify a positive time vector.');
    end

    if time(2) < time(1) + minIntTime
        error('interpolate:TooShortTimeInterval', ...
              'Please specify a larger time interval to interpolate on.');
    end

    if nRef > 1 && data.time(1) + time(1) > ref(end) - minIntTime
        error('interpolate:TimeOutOfRange', ...
              'Please specify a lower time bound that %s %s', ...
              'sufficiently falls within the reference time interval ', ...
              '(at least minIntTime s).');
    elseif data.time(1) + time(1) > data.time(end) - minIntTime
        error('interpolate:TimeOutOfRange', ...
              'Please specify a lower time bound that %s %s', ...
              'sufficiently falls within the recorded time interval ', ...
              '(at least minIntTime s).');
    end

    if nRef > 1 && ref(1) + time(2) > ref(end)
        warning('interpolate:TimeOutOfRange', ...
                'The provided upper time bound falls outside %s %s %s', ...
                'the reference time interval.', ...
                'The reference signal will be truncated at the end ', ...
                'of the reference time sequence.');
    elseif data.time(1) + time(2) > data.time(end) && ...
           data.time(end) < ref(end)
        warning('interpolate:TimeOutOfRange', ...
                'The provided upper time bound falls outside %s %s %s', ...
                'the recorded time interval.', ...
                'The interpolated signal will be truncated at the ', ...
                'end of the data.time sequence.');
    end
end
%--------------------------------------------------------------------------


%------------- Determine start and end time of interpolation --------------
if nargin == 2
    if nRef == 1
        startTime = data.time(1);
        endTime = data.time(end);
    else
        startTime = max(ref(1),data.time(1));
        endTime = min(ref(end),data.time(end));
    end
elseif nargin == 3
    startTime = data.time(1) + time(1);
    endTime = data.time(1) + time(2);
    if endTime > data.time(end)
        endTime = data.time(end);
    end
    if nRef > 1 && endTime > ref(end)
        endTime = ref(end);
    end
end
%--------------------------------------------------------------------------


%----------- Construct a reference signal from a sampling time ------------
% Ensure proper interpolation
% (only interpolate on times that are contained in data):
% 1. intData.time(1)   >= startTime
% 2. intData.time(end) <= endTime
%
% In case of nRef == 1 (sampling time): construct time sequence within
% interval [startTime, endTime].
% In case of nRef > 1 (time sequence): constrain to interval [startTime,
% endTime].

intData.time = ref;
if nRef == 1
    sampleTime = ref;
    
    % Determine amount of samples, while taking floating-point inaccuracies
    % into account
    tempAmount = (endTime - startTime)/ref;
    tempDiff = abs(tempAmount - floor(tempAmount));
    if tempDiff > 1 - timeThres
        nTime = floor(tempAmount) + 2;
    else
        nTime = floor(tempAmount) + 1;
    end
    
    intData.time = zeros(1,nTime);
    intData.time(1) = startTime;
    for i = 2:nTime
        intData.time(i) = intData.time(i-1) + sampleTime;
    end
else
    if intData.time(1) < startTime
        i = 2;
        while (startTime - intData.time(i)) > timeThres
            i = i + 1;
        end
        intData.time = intData.time(i:end);
    end
    if intData.time(end) > endTime
        i = length(intData.time) - 1;
        while (intData.time(i) - endTime) > timeThres
            i = i - 1;
        end
        intData.time = intData.time(1:i);
    end
end
%--------------------------------------------------------------------------


%------------------------------ Interpolate -------------------------------
nRef = length(intData.time);
nData = length(data.time);
dataIdx = 1;
for refIdx = 1:nRef
    % Get next time index where data.time >= ref
    while data.time(dataIdx) < intData.time(refIdx)
        if dataIdx == nData
            break;
        end
        dataIdx = dataIdx + 1;
    end
    
    % If data.time == ref: no interpolation needed, just use that sample
    if abs(data.time(dataIdx) - intData.time(refIdx)) < timeThres
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
%--------------------------------------------------------------------------
end