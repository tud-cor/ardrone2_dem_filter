function createCustomInterpolateTestDataMat()
% Initialisation
close all;

%------------------------------ Parameters --------------------------------
% Accepted floating-point accuracy
timeThres = 1e-10;

% data parameters
dataFreq = 1;
dataDur = 40;
dataOffset = 110;

% int parameters
refFreq = [0.951, 1, 1.21];

% time parameters
time = [3,7];
%--------------------------------------------------------------------------
% Construction of data
data.time = createTimeSequence(dataFreq,dataDur,dataOffset,1);
data.timeFixed = fixDups(data.time);
nData = length(data.timeFixed);
data.value = 100*randi([0,10],1,nData);

for intIdx = 1:length(refFreq)
    % Construction of int
    sampleTime = 1/refFreq(intIdx);
    % For tests starting from t = dataOffset
    tempTime = data.timeFixed(1);
    i = 1;
    while abs(tempTime - data.timeFixed(end)) < timeThres || ...
          tempTime < data.timeFixed(end)
        int(intIdx).time(i) = tempTime;
        i = i + 1;
        tempTime = tempTime + sampleTime;
    end
    % For tests starting from t = dataOffset + time(1)
    tempTime = data.timeFixed(1) + time(1);
    endTime = data.timeFixed(1) + time(2);
    i = 1;
    while abs(tempTime - endTime) < timeThres || tempTime < endTime
        int(intIdx).timeSmall(i) = tempTime;
        i = i + 1;
        tempTime = tempTime + sampleTime;
    end

    % Interpolation for tests starting from t = dataOffset
    nInt = length(int(intIdx).time);
    int(intIdx).value = zeros(1,nInt);
    j = 1;
    for i = 1:nInt
        % Find j for which data.timeFixed > int.time
        while abs(int(intIdx).time(i) - data.timeFixed(j)) < timeThres ...
              || int(intIdx).time(i) > data.timeFixed(j)
            if (j == nData)
                break;
            end
            j = j + 1;
        end

        % Interpolate data:
        % - data.timeFixed = int.time => just use that value
        % - data.timeFixed(j-1) <= int.time < data.timeFixed(j) => linearly
        %   interpolate
        if abs(int(intIdx).time(i) - data.timeFixed(j-1)) < timeThres
            int(intIdx).value(i) = data.value(j-1);
        else
            int(intIdx).value(i) = data.value(j-1) + ...
                           (data.value(j) - data.value(j-1))/ ...
                           (data.timeFixed(j) - data.timeFixed(j-1))* ...
                           (int(intIdx).time(i) - data.timeFixed(j-1));
        end
    end

    % Interpolation for tests starting from t = dataOffset + time(1)
    nIntSmall = length(int(intIdx).timeSmall);
    int(intIdx).valueSmall = zeros(1,nIntSmall);
    j = 1;
    for i = 1:nIntSmall
        % Find j for which data.timeFixed > int.time
        while abs(int(intIdx).timeSmall(i) - data.timeFixed(j)) < timeThres || ...
              int(intIdx).timeSmall(i) > data.timeFixed(j)
            if (j == nData)
                break;
            end
            j = j + 1;
        end

        % Interpolate data:
        % - data.timeFixed = int.timeSmall => just use that value
        % - data.timeFixed(j-1) <= int.timeSmall < data.timeFixed(j) => 
        %   linearly interpolate
        if abs(int(intIdx).timeSmall(i) - data.timeFixed(j-1)) < timeThres
            int(intIdx).valueSmall(i) = data.value(j-1);
        else
            int(intIdx).valueSmall(i) = data.value(j-1) + ...
                           (data.value(j) - data.value(j-1))/ ...
                           (data.timeFixed(j) - data.timeFixed(j-1))* ...
                           (int(intIdx).timeSmall(i) - data.timeFixed(j-1));
        end
    end

    % Plot original data and interpolated data
    figure(2*intIdx - 1);
    plot(data.timeFixed, data.value, '-o');
    hold on;
    plot(int(intIdx).time, int(intIdx).value, '-o');
    figure(2*intIdx);
    plot(data.timeFixed, data.value, '-o');
    hold on;
    plot(int(intIdx).timeSmall, int(intIdx).valueSmall, '-o');
end

% Save results in a .mat file
save('customInterpolateTestData.mat', 'data', 'dataFreq', 'dataDur', ...
     'dataOffset', 'int', 'refFreq');
end