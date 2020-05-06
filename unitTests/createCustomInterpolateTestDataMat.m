function createCustomInterpolateTestDataMat()
timeThres = 1e-10;

dataFreq = 1000;
dataDur = 10;
dataOffset = 0.11;
data.time = createTimeSequence(dataFreq,dataDur,dataOffset,1);
data.time_fixed = fixDups(data.time);
nData = length(data.time_fixed);
data.value = 100*randi([0,10],1,nData);

refFreq = 951;
sampleTime = 1/refFreq;
tempTime = data.time_fixed(1);
i = 1;
while abs(tempTime - data.time_fixed(end)) < timeThres || ...
      tempTime < data.time_fixed(end)
    int.time(i) = tempTime;
    i = i + 1;
    tempTime = tempTime + sampleTime;
end

nInt = length(int.time);
int.value = zeros(1,nInt);
j = 1;
for i = 1:nInt
    % Find j for which data.time_fixed > int.time
    while abs(int.time(i) - data.time_fixed(j)) < timeThres || ...
          int.time(i) > data.time_fixed(j)
        if (j == nData)
            break;
        end
        j = j + 1;
    end
    
    if abs(int.time(i) - data.time_fixed(j-1)) < timeThres
        int.value(i) = data.value(j-1);
    else
        int.value(i) = data.value(j-1) + ...
                       (data.value(j) - data.value(j-1))/ ...
                       (data.time_fixed(j) - data.time_fixed(j-1))* ...
                       (int.time(i) - data.time_fixed(j-1));
    end
end

close all;
plot(data.time_fixed, data.value, '-o');
hold on;
plot(int.time, int.value, '-o');

save('customInterpolateTestData.mat', 'data', 'dataFreq', 'dataDur', ...
     'dataOffset', 'int', 'refFreq');
end