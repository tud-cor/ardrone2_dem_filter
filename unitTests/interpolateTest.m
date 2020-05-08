%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% interpolate unit test file
%
% This file contains test functions according to the MATLAB unit test
% framework to properly check the functionality of function interpolate.
%
% Author: Dennis Benders
% Last edited: 08.05.2020
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main test function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function tests = interpolateTest
tests = functiontests(localfunctions);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End main test function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Local test functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------- Errors and warnings --------------------------
function testTooFewArguments(testCase)
ref = 0.001;
act = @()interpolate(ref);
exp = 'interpolate:TooFewArguments';
verifyError(testCase,act,exp);
end

function testTimeNegative(testCase)
ref = 0.001;
data.time = [0.001, 0.002, 0.002, 0.004, 0.004, 0.006, 0.007, 0.008];
data.value = [10.0, 2.0, 5.0, 1.0, 3.0, 6.0, 9.5, 2.5];
time = [-0.001,0.0];
act = @()interpolate(ref, data, time);
exp = 'interpolate:TimeNegative';
verifyError(testCase,act,exp);
end

function testTimeNegative2(testCase)
ref = 0.001;
data.time = [0.001, 0.002, 0.002, 0.004, 0.004, 0.006, 0.007, 0.008];
data.value = [10.0, 2.0, 5.0, 1.0, 3.0, 6.0, 9.5, 2.5];
time = [0.0,-0.001];
act = @()interpolate(ref, data, time);
exp = 'interpolate:TimeNegative';
verifyError(testCase,act,exp);
end

function testTimeNegative3(testCase)
ref = 0.001;
data.time = [0.001, 0.002, 0.002, 0.004, 0.004, 0.006, 0.007, 0.008];
data.value = [10.0, 2.0, 5.0, 1.0, 3.0, 6.0, 9.5, 2.5];
time = [-0.001,-0.001];
act = @()interpolate(ref, data, time);
exp = 'interpolate:TimeNegative';
verifyError(testCase,act,exp);
end

function testTooShortTimeInterval(testCase)
ref = 0.001;
data.time = [0.001, 0.002, 0.002, 0.004, 0.004, 0.006, 0.007, 0.008];
data.value = [10.0, 2.0, 5.0, 1.0, 3.0, 6.0, 9.5, 2.5];
time = [0.001,0.0];
act = @()interpolate(ref, data, time);
exp = 'interpolate:TooShortTimeInterval';
verifyError(testCase,act,exp);
end

function testTooShortTimeInterval2(testCase)
ref = 0.001;
data.time = [0.001, 0.002, 0.002, 0.004, 0.004, 0.006, 0.007, 0.008];
data.value = [10.0, 2.0, 5.0, 1.0, 3.0, 6.0, 9.5, 2.5];
time = [0.0,0.999];
act = @()interpolate(ref, data, time);
exp = 'interpolate:TooShortTimeInterval';
verifyError(testCase,act,exp);
end

function testTimeOutOfRange(testCase)
ref = createTimeSequence(50,5,0);
data.time = createTimeSequence(1000,10,0,1);
data.value = createRandomValueSequence(length(data.time),0,30);
time = [4.1,12];
act = @()interpolate(ref, data, time);
exp = 'interpolate:TimeOutOfRange';
verifyError(testCase,act,exp);
end

function testTimeOutOfRange2(testCase)
ref = createTimeSequence(50,11,0);
data.time = createTimeSequence(1000,10,0,1);
data.value = createRandomValueSequence(length(data.time),0,30);
time = [9.1,12];
act = @()interpolate(ref, data, time);
exp = 'interpolate:TimeOutOfRange';
verifyError(testCase,act,exp);
end

function testTimeOutOfRange3(testCase)
ref = createTimeSequence(50,5,0);
data.time = createTimeSequence(1000,10,0,1);
data.value = createRandomValueSequence(length(data.time),0,30);
time = [4.0,5.1];
act = @()interpolate(ref, data, time);
exp = 'interpolate:TimeOutOfRange';
verifyWarning(testCase,act,exp);
end

function testTimeOutOfRange4(testCase)
ref = createTimeSequence(50,11,0);
data.time = createTimeSequence(1000,10,0,1);
data.value = createRandomValueSequence(length(data.time),0,30);
time = [9.0,11];
act = @()interpolate(ref, data, time);
exp = 'interpolate:TimeOutOfRange';
verifyWarning(testCase,act,exp);
end
%--------------------------------------------------------------------------

%------------------------ ref and data same params ------------------------
function testNref1TimeSmall(testCase)
refFreq = 1000;
ref = 1/refFreq;
dataFreq = 1000;
dataDur = 10;
dataOffset = 0.1;
data.time = createTimeSequence(dataFreq,dataDur,dataOffset,1);
data.value = createRandomValueSequence(length(data.time),0,30);
time = [3,7];
act = interpolate(ref, data, time);
exp.time = createTimeSequence(dataFreq,time(2)-time(1), ...
                              time(1)+dataOffset,0);
exp.value = data.value(time(1)*dataFreq+1:time(2)*dataFreq+1);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testRefEqualDataTimeSmall(testCase)
refFreq = 1000;
refDur = 10;
refOffset = 0.1;
ref = createTimeSequence(refFreq,refDur,refOffset,1);
data.time = ref;
data.value = createRandomValueSequence(length(data.time),0,30);
time = [3,7];
act = interpolate(ref, data, time);
exp.time = createTimeSequence(refFreq,time(2)-time(1), ...
                              time(1)+refOffset,0);
exp.value = data.value(time(1)*refFreq+1:time(2)*refFreq+1);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testRefDataSameParamTimeSmall(testCase)
refFreq = 1000;
refDur = 10;
refOffset = 0.1;
ref = createTimeSequence(refFreq,refDur,refOffset,1);
dataFreq = refFreq;
dataDur = refDur;
dataOffset = refOffset;
data.time = createTimeSequence(dataFreq,dataDur,dataOffset,1);
data.value = createRandomValueSequence(length(data.time),0,30);
time = [3,7];
act = interpolate(ref, data, time);
exp.time = createTimeSequence(dataFreq,time(2)-time(1), ...
                              time(1)+dataOffset,0);
exp.value = data.value(time(1)*dataFreq+1:time(2)*dataFreq+1);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end
%--------------------------------------------------------------------------

%--------------------- Random with refFreq = n*dataFreq -------------------
function testRandomSameFreqWithoutTime(testCase)
load randomInterpolateTestData.mat data dataFreq dataDur dataOffset;
refFreq = dataFreq;
ref = 1/refFreq;
act = interpolate(ref, data);
exp.time = createTimeSequence(dataFreq,dataDur,dataOffset,0);
exp.value = data.value;
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testRandomSameFreqWithTime(testCase)
load randomInterpolateTestData.mat data dataFreq dataOffset;
refFreq = dataFreq;
ref = 1/refFreq;
time = [0,7];
act = interpolate(ref, data, time);
exp.time = createTimeSequence(dataFreq,time(2)-time(1), ...
                              time(1)+dataOffset,0);
exp.value = data.value(time(1)*dataFreq+1:time(2)*dataFreq+1);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testRandomSameFreqTimeSmall(testCase)
load randomInterpolateTestData.mat data dataFreq dataOffset;
refFreq = dataFreq;
ref = 1/refFreq;
time = [3,7];
act = interpolate(ref, data, time);
exp.time = createTimeSequence(dataFreq,time(2)-time(1), ...
                              time(1)+dataOffset,0);
exp.value = data.value(time(1)*dataFreq+1:time(2)*dataFreq+1);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testRandomLowMultFreqWithoutTime(testCase)
load randomInterpolateTestData.mat data dataFreq dataDur dataOffset;
refFreq = 50;
ref = 1/refFreq;
dataRefRatio = dataFreq/refFreq;
act = interpolate(ref, data);
tempTime = createTimeSequence(dataFreq,dataDur,dataOffset,0);
exp.time = tempTime(1:dataRefRatio:end);
exp.value = data.value(1:dataRefRatio:end);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testRandomLowMultFreqWithTime(testCase)
load randomInterpolateTestData.mat data dataFreq dataOffset;
refFreq = 50;
ref = 1/refFreq;
dataRefRatio = dataFreq/refFreq;
time = [0,7];
act = interpolate(ref, data, time);
tempTime = createTimeSequence(dataFreq,time(2)-time(1), ...
                              time(1)+dataOffset,0);
exp.time = tempTime(1:dataRefRatio:end);
exp.value = data.value(time(1)*dataFreq+1:dataRefRatio:time(2)*dataFreq+1);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testRandomLowMultFreqTimeSmall(testCase)
load randomInterpolateTestData.mat data dataFreq dataOffset;
refFreq = 50;
ref = 1/refFreq;
dataRefRatio = dataFreq/refFreq;
time = [3,7];
act = interpolate(ref, data, time);
tempTime = createTimeSequence(dataFreq,time(2)-time(1), ...
                              time(1)+dataOffset,0);
exp.time = tempTime(1:dataRefRatio:end);
exp.value = data.value(time(1)*dataFreq+1:dataRefRatio:time(2)*dataFreq+1);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testRandomHighMultFreqWithoutTime(testCase)
load randomInterpolateTestData.mat data dataFreq dataDur dataOffset;
refFreq = 3000;
ref = 1/refFreq;
refDataRatio = refFreq/dataFreq;
act = interpolate(ref, data);
exp.time = createTimeSequence(refFreq,dataDur,dataOffset,0);
exp.value = simpleInterpolate(data.value,refDataRatio);
verifyEqual(testCase,act,exp,'AbsTol',1e-6);
end

function testRandomHighMultFreqWithTime(testCase)
load randomInterpolateTestData.mat data dataFreq dataOffset;
refFreq = 3000;
ref = 1/refFreq;
refDataRatio = refFreq/dataFreq;
time = [0,7];
act = interpolate(ref, data, time);
exp.time = createTimeSequence(refFreq,time(2)-time(1), ...
                              time(1)+dataOffset,0);
tempValue = simpleInterpolate(data.value,refDataRatio);
exp.value = tempValue(time(1)*refFreq+1:time(2)*refFreq+1);
verifyEqual(testCase,act,exp,'AbsTol',1e-6);
end

function testRandomHighMultFreqTimeSmall(testCase)
load randomInterpolateTestData.mat data dataFreq dataOffset;
refFreq = 3000;
ref = 1/refFreq;
refDataRatio = refFreq/dataFreq;
time = [3,7];
act = interpolate(ref, data, time);
exp.time = createTimeSequence(refFreq,time(2)-time(1), ...
                              time(1)+dataOffset,0);
tempValue = simpleInterpolate(data.value,refDataRatio);
exp.value = tempValue(time(1)*refFreq+1:time(2)*refFreq+1);
verifyEqual(testCase,act,exp,'AbsTol',1e-6);
end
%--------------------------------------------------------------------------

%---------------------- Custom with arbitrary refFreq ---------------------
% customInterpolateTestData.mat can be visually inspected using function 
% createCustomInterpolateTestDataMat. This shows that interpolation is done
% properly and the results can be used to test the interpolate function.
% Furthermore, this function has proven succesful for dataFreq = 1 and
% refFreq = 0.951 (equivalent to using the more realistic frequencies 
% 1000 Hz and 951 Hz, respectively, but used to not let the interpolate
% function throw the TooShortTimeInterval error) by checking it with 
% calculations by hand on a relatively small time interval.

function testCustomLowFreqWithoutTime(testCase)
load customInterpolateTestData.mat data refFreq int;
ref = 1/refFreq(1);
act = interpolate(ref, data);
exp.time = int(1).time;
exp.value = int(1).value;
verifyEqual(testCase,act,exp,'AbsTol',1e-6);
end

function testCustomLowFreqWithTime(testCase)
load customInterpolateTestData.mat data dataOffset refFreq int;
timeThres = 1e-10;
ref = 1/refFreq(1);
time = [0,7];
act = interpolate(ref, data, time);
l = length(int(1).time);
for i = 1:l
    if abs(int(1).time(i) - (dataOffset + time(1))) < timeThres || ...
       int(1).time(i) > (dataOffset + time(1))
        break;
    end
end
for j = i:l
    if int(1).time(j) > (dataOffset + time(2))
        break;
    end
end
j = j - 1;
exp.time = int(1).time(i:j);
exp.value = int(1).value(i:j);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testCustomLowFreqTimeSmall(testCase)
load customInterpolateTestData.mat data refFreq int;
ref = 1/refFreq(1);
time = [3,7];
act = interpolate(ref, data, time);
exp.time = int(1).timeSmall;
exp.value = int(1).valueSmall;
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testCustomLowFreqSeqTimeSmall(testCase)
load customInterpolateTestData.mat data dataOffset int;
timeThres = 1e-10;
ref = int(1).time;
time = [3,7];
act = interpolate(ref, data, time);

l = length(int(1).time);
for i = 1:l
    if abs(int(1).time(i) - (dataOffset + time(1))) < timeThres || ...
       int(1).time(i) > (dataOffset + time(1))
        break;
    end
end
for j = i:l
    if int(1).time(j) > (dataOffset + time(2))
        break;
    end
end
j = j - 1;
exp.time = int(1).time(i:j);
exp.value = int(1).value(i:j);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testCustomSameFreqWithoutTime(testCase)
load customInterpolateTestData.mat data refFreq int;
ref = 1/refFreq(2);
act = interpolate(ref, data);
exp.time = int(2).time;
exp.value = int(2).value;
verifyEqual(testCase,act,exp,'AbsTol',1e-6);
end

function testCustomSameFreqWithTime(testCase)
load customInterpolateTestData.mat data dataOffset refFreq int;
timeThres = 1e-10;
ref = 1/refFreq(2);
time = [0,7];
act = interpolate(ref, data, time);
l = length(int(2).time);
for i = 1:l
    if abs(int(2).time(i) - (dataOffset + time(1))) < timeThres || ...
       int(2).time(i) > (dataOffset + time(1))
        break;
    end
end
for j = i:l
    if int(2).time(j) > (dataOffset + time(2))
        break;
    end
end
j = j - 1;
exp.time = int(2).time(i:j);
exp.value = int(2).value(i:j);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testCustomSameFreqTimeSmall(testCase)
load customInterpolateTestData.mat data refFreq int;
ref = 1/refFreq(2);
time = [3,7];
act = interpolate(ref, data, time);
exp.time = int(2).timeSmall;
exp.value = int(2).valueSmall;
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testCustomSameFreqSeqTimeSmall(testCase)
load customInterpolateTestData.mat data dataOffset int;
timeThres = 1e-10;
ref = int(2).time;
time = [3,7];
act = interpolate(ref, data, time);

l = length(int(2).time);
for i = 1:l
    if abs(int(2).time(i) - (dataOffset + time(1))) < timeThres || ...
       int(2).time(i) > (dataOffset + time(1))
        break;
    end
end
for j = i:l
    if int(2).time(j) > (dataOffset + time(2))
        break;
    end
end
j = j - 1;
exp.time = int(2).time(i:j);
exp.value = int(2).value(i:j);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testCustomHighFreqWithoutTime(testCase)
load customInterpolateTestData.mat data refFreq int;
ref = 1/refFreq(3);
act = interpolate(ref, data);
exp.time = int(3).time;
exp.value = int(3).value;
verifyEqual(testCase,act,exp,'AbsTol',1e-6);
end

function testCustomHighFreqWithTime(testCase)
load customInterpolateTestData.mat data dataOffset refFreq int;
timeThres = 1e-10;
ref = 1/refFreq(3);
time = [0,7];
act = interpolate(ref, data, time);
l = length(int(3).time);
for i = 1:l
    if abs(int(3).time(i) - (dataOffset + time(1))) < timeThres || ...
       int(3).time(i) > (dataOffset + time(1))
        break;
    end
end
for j = i:l
    if int(3).time(j) > (dataOffset + time(2))
        break;
    end
end
j = j - 1;
exp.time = int(3).time(i:j);
exp.value = int(3).value(i:j);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testCustomHighFreqTimeSmall(testCase)
load customInterpolateTestData.mat data refFreq int;
ref = 1/refFreq(3);
time = [3,7];
act = interpolate(ref, data, time);
exp.time = int(3).timeSmall;
exp.value = int(3).valueSmall;
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testCustomHighFreqSeqTimeSmall(testCase)
load customInterpolateTestData.mat data dataOffset int;
timeThres = 1e-10;
ref = int(3).time;
time = [3,7];
act = interpolate(ref, data, time);

l = length(int(3).time);
for i = 1:l
    if abs(int(3).time(i) - (dataOffset + time(1))) < timeThres || ...
       int(3).time(i) > (dataOffset + time(1))
        break;
    end
end
for j = i:l
    if int(3).time(j) > (dataOffset + time(2))
        break;
    end
end
j = j - 1;
exp.time = int(3).time(i:j);
exp.value = int(3).value(i:j);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end
%--------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End local test functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Local functions used in test functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = createTimeSequence(sampleFreq, dur, offset, putDups)
% CREATETIMESEQUENCE Create an array with linearly increasing numbers,
% starting from offset, with length sampleFreq*dur and with potentially
% duplicated time numbers if indicated so by putDups.
%
%   Author: Dennis Benders
%   Last edited: 07.05.2020
%
%   Input:  sampleFreq  sample frequency (inverse of array increments)
%           dur         length of time sequence starting from offset
%           offset      start of time sequence in s
%           putDups     if 1: put duplications in the time sequence, while
%                       maintaining the increase. if 0: forget about
%                       duplications
%
%   Output: output      array with linearly increasing time numbers,
%                       potentially with duplicated times
%
%   For usage, see local test functions above.


%------------------------------ Check inputs ------------------------------
if nargin < 4
    putDups = 0;
end
%--------------------------------------------------------------------------


%----------------------------- Set parameters -----------------------------
sampleTime = 1/sampleFreq;
n = sampleFreq*dur + 1;
%--------------------------------------------------------------------------


%---------------------------- Initialise output ---------------------------
output = zeros(1,n);
output(1) = offset;
%--------------------------------------------------------------------------

%-------------------- Initialise indices of duplications ------------------
if putDups
    randSeq = round(rand(n,1));
end
%--------------------------------------------------------------------------


%---------------------------- Construct output ----------------------------
for i = 2:n
    if putDups && randSeq(i)
        output(i) = output(i-1);
    else
        output(i) = output(1) + (i-1)*sampleTime;
    end
end
%--------------------------------------------------------------------------
end



function output = createRandomValueSequence(n, min, max)
% CREATERANDOMVALUESEQUENCE Create an array of length n with random numbers
% picked from a uniform distribution between min and max.
%
%   Author: Dennis Benders
%   Last edited: 07.05.2020
%
%   Input:  n       length of array
%           min     lower bound of uniform distribution from which the
%                   random numbers are picked
%           max     upper bound of uniform distribution from which the
%                   random numbers are picked
%
%   Output: output  array of length n with randomly picked numbers between
%                   min and max
%
%   For usage, see local test functions above.


%---------------------------- Construct output ----------------------------
output = min + (max - min)*rand(1,n);
%--------------------------------------------------------------------------
end



function output = simpleInterpolate(inputArray, refDataRatio)
% SIMPLEINTERPOLATE Linearly interpolate data in inputArray with a
% frequency that is refDataRatio higher than the sample frequency with
% which the data in inputArray is sampled.
%
%   Author: Dennis Benders
%   Last edited: 07.05.2020
%
%   Input:  inputArray      array of data that needs to be interpolated
%           refDataRatio    ratio of sampling frequency of original data in
%                           inputArray and the desired frequency of data
%                           after interpolation
%
%   Output: output          array with interpolated data having a sampling
%                           frequency that is refDataRatio times higher
%                           than the frequency at which the input data is
%                           sampled
%
%   For usage, see local test functions above.


%---------------------------- Construct output ----------------------------
l = length(inputArray);
output = zeros(1,(l-1)*refDataRatio+1);
for i = 1:l-1
    for j = 1:refDataRatio
        output(refDataRatio*(i-1)+j) = inputArray(i) + ...
            (j-1)/refDataRatio*(inputArray(i+1) - inputArray(i));
    end
end
output(end) = inputArray(end);
%--------------------------------------------------------------------------
end



function createRandomInterpolateTestDataMat()
% CREATERANDOMINTERPOLATETESTDATAMAT Create a .mat file containing test
% data (time as well as value sequence) with a certain sampling frequency,
% duration and starting time.
%
%   Author: Dennis Benders
%   Last edited: 07.05.2020
%
%   Input:  none
%
%   Output: randomInterpolateTestData	.mat file with time (including
%                                       duplications) and value sequences
%                                       and parameters with which the data
%                                       was generated
%
%   For usage of the generated .mat file, see local test functions above.


%--------------------------- Set data parameters --------------------------
dataFreq = 1000;
dataDur = 10;
dataOffset = 0.12;
%--------------------------------------------------------------------------


%----------------------------- Construct data -----------------------------
% Determine length of data.time and data.value arrays
n = dataFreq*dataDur + 1;

% Create data.time and data.value arrays
data.time = createTimeSequence(dataFreq,dataDur,dataOffset,1);
data.value = createRandomValueSequence(n,0,30);
%--------------------------------------------------------------------------


%-------------------------------- Plot data -------------------------------
plot(data.time, data.value);
%--------------------------------------------------------------------------


%----------------------- Save results in a .mat file ----------------------
save('randomInterpolateTestData.mat', 'data', 'dataFreq', 'dataDur', ...
     'dataOffset');
%--------------------------------------------------------------------------
end



function createCustomInterpolateTestDataMat()
% CREATECUSTOMINTERPOLATETESTDATAMAT Create a .mat file containing test
% data (time as well as value sequence) as well as interpolated test data
% to compare with, generated at three different interpolation sampling
% frequencies.
%
%   Author: Dennis Benders
%   Last edited: 07.05.2020
%
%   Input:  none
%
%   Output: customInterpolateTestData	.mat file with time (including
%                                       duplications) and value sequences
%                                       for test as well as interpolated
%                                       data, including the parameters with
%                                       which the data was generated
%
%   For usage of the generated .mat file, see local test functions above.


%----------------------------- Set parameters -----------------------------
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


%----------------------------- Construct data -----------------------------
data.time = createTimeSequence(dataFreq,dataDur,dataOffset,1);
data.timeFixed = fixDups(data.time);
nData = length(data.timeFixed);
data.value = 100*randi([0,10],1,nData);
%--------------------------------------------------------------------------


%------------------ Construct and plot interpolated data ------------------
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
        % - data.timeFixed(j-1) <= int.time < data.timeFixed(j) =>
        %   linearly interpolate
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
%--------------------------------------------------------------------------


%----------------------- Save results in a .mat file ----------------------
save('customInterpolateTestData.mat', 'data', 'dataFreq', 'dataDur', ...
     'dataOffset', 'int', 'refFreq');
%--------------------------------------------------------------------------
end
%--------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End local functions used in test functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%