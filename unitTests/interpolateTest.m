% Main test function
function tests = interpolateTest
tests = functiontests(localfunctions);
end

%- ref = sampling time
%- ref = time sequence
%- duplicated times in data.time signal
%- ref with different lengths
%- time array with all kinds of warning and error forms
%- (essentially all kinds of variations of time shifting in ref, data.time
%and time)

% Local test functions
% function testTooFewArguments(testCase)
% ref = 0.001;
% act = @()interpolate(ref);
% exp = 'interpolate:TooFewArguments';
% verifyError(testCase,act,exp);
% end
% 
% function testTimeNegative(testCase)
% ref = 0.001;
% data.time = [0.001, 0.002, 0.002, 0.004, 0.004, 0.006, 0.007, 0.008];
% data.value = [10.0, 2.0, 5.0, 1.0, 3.0, 6.0, 9.5, 2.5];
% time = [-0.001,0.0];
% act = @()interpolate(ref, data, time);
% exp = 'interpolate:TimeNegative';
% verifyError(testCase,act,exp);
% end
% 
% function testTimeNegative2(testCase)
% ref = 0.001;
% data.time = [0.001, 0.002, 0.002, 0.004, 0.004, 0.006, 0.007, 0.008];
% data.value = [10.0, 2.0, 5.0, 1.0, 3.0, 6.0, 9.5, 2.5];
% time = [0.0,-0.001];
% act = @()interpolate(ref, data, time);
% exp = 'interpolate:TimeNegative';
% verifyError(testCase,act,exp);
% end
% 
% function testTimeNegative3(testCase)
% ref = 0.001;
% data.time = [0.001, 0.002, 0.002, 0.004, 0.004, 0.006, 0.007, 0.008];
% data.value = [10.0, 2.0, 5.0, 1.0, 3.0, 6.0, 9.5, 2.5];
% time = [-0.001,-0.001];
% act = @()interpolate(ref, data, time);
% exp = 'interpolate:TimeNegative';
% verifyError(testCase,act,exp);
% end
% 
% function testTooShortTimeInterval(testCase)
% ref = 0.001;
% data.time = [0.001, 0.002, 0.002, 0.004, 0.004, 0.006, 0.007, 0.008];
% data.value = [10.0, 2.0, 5.0, 1.0, 3.0, 6.0, 9.5, 2.5];
% time = [0.001,0.0];
% act = @()interpolate(ref, data, time);
% exp = 'interpolate:TooShortTimeInterval';
% verifyError(testCase,act,exp);
% end
% 
% function testTooShortTimeInterval2(testCase)
% ref = 0.001;
% data.time = [0.001, 0.002, 0.002, 0.004, 0.004, 0.006, 0.007, 0.008];
% data.value = [10.0, 2.0, 5.0, 1.0, 3.0, 6.0, 9.5, 2.5];
% time = [0.0,0.999];
% act = @()interpolate(ref, data, time);
% exp = 'interpolate:TooShortTimeInterval';
% verifyError(testCase,act,exp);
% end
% 
% function testTimeOutOfRange(testCase)
% ref = createTimeSequence(50,5,0);
% data.time = createTimeSequence(1000,10,0,1);
% data.value = createRandomValueSequence(length(data.time),0,30);
% time = [4.1,12];
% act = @()interpolate(ref, data, time);
% exp = 'interpolate:TimeOutOfRange';
% verifyError(testCase,act,exp);
% end
% 
% function testTimeOutOfRange2(testCase)
% ref = createTimeSequence(50,11,0);
% data.time = createTimeSequence(1000,10,0,1);
% data.value = createRandomValueSequence(length(data.time),0,30);
% time = [9.1,12];
% act = @()interpolate(ref, data, time);
% exp = 'interpolate:TimeOutOfRange';
% verifyError(testCase,act,exp);
% end
% 
% function testTimeOutOfRange3(testCase)
% ref = createTimeSequence(50,5,0);
% data.time = createTimeSequence(1000,10,0,1);
% data.value = createRandomValueSequence(length(data.time),0,30);
% time = [4.0,5.1];
% act = @()interpolate(ref, data, time);
% exp = 'interpolate:TimeOutOfRange';
% verifyWarning(testCase,act,exp);
% end
% 
% function testTimeOutOfRange4(testCase)
% ref = createTimeSequence(50,11,0);
% data.time = createTimeSequence(1000,10,0,1);
% data.value = createRandomValueSequence(length(data.time),0,30);
% time = [9.0,11];
% act = @()interpolate(ref, data, time);
% exp = 'interpolate:TimeOutOfRange';
% verifyWarning(testCase,act,exp);
% end
% 
% function testNref1TimeSmall(testCase)
% refFreq = 1000;
% ref = 1/refFreq;
% dataFreq = 1000;
% dataDur = 10;
% dataOffset = 0.1;
% data.time = createTimeSequence(dataFreq,dataDur,dataOffset,1);
% data.value = createRandomValueSequence(length(data.time),0,30);
% time = [3,7];
% act = interpolate(ref, data, time);
% exp.time = createTimeSequence(dataFreq,time(2)-time(1), ...
%                               time(1)+dataOffset,0);
% exp.value = data.value(time(1)*dataFreq+1:time(2)*dataFreq+1);
% verifyEqual(testCase,act,exp,'AbsTol',1e-7);
% end
% 
% function testRefEqualDataTimeSmall(testCase)
% refFreq = 1000;
% refDur = 10;
% refOffset = 0.1;
% ref = createTimeSequence(refFreq,refDur,refOffset,1);
% data.time = ref;
% data.value = createRandomValueSequence(length(data.time),0,30);
% time = [3,7];
% act = interpolate(ref, data, time);
% exp.time = createTimeSequence(refFreq,time(2)-time(1), ...
%                               time(1)+refOffset,0);
% exp.value = data.value(time(1)*refFreq+1:time(2)*refFreq+1);
% verifyEqual(testCase,act,exp,'AbsTol',1e-7);
% end
% 
% function testRefDataSameParamTimeSmall(testCase)
% refFreq = 1000;
% refDur = 10;
% refOffset = 0.1;
% ref = createTimeSequence(refFreq,refDur,refOffset,1);
% dataFreq = refFreq;
% dataDur = refDur;
% dataOffset = refOffset;
% data.time = createTimeSequence(dataFreq,dataDur,dataOffset,1);
% data.value = createRandomValueSequence(length(data.time),0,30);
% time = [3,7];
% act = interpolate(ref, data, time);
% exp.time = createTimeSequence(dataFreq,time(2)-time(1), ...
%                               time(1)+dataOffset,0);
% exp.value = data.value(time(1)*dataFreq+1:time(2)*dataFreq+1);
% verifyEqual(testCase,act,exp,'AbsTol',1e-7);
% end
% % 
% function testRandomSameFreqWithoutTime(testCase)
% load randomInterpolateTestData.mat data dataFreq dataDur dataOffset;
% refFreq = dataFreq;
% ref = 1/refFreq;
% act = interpolate(ref, data);
% exp.time = createTimeSequence(dataFreq,dataDur,dataOffset,0);
% exp.value = data.value;
% verifyEqual(testCase,act,exp,'AbsTol',1e-7);
% end
% 
% % TODO: create a time sequence for ref using this test function!
% function testRandomSameFreqWithTime(testCase)
% load randomInterpolateTestData.mat data dataFreq dataOffset;
% refFreq = dataFreq;
% ref = 1/refFreq;
% time = [0,7];
% act = interpolate(ref, data, time);
% exp.time = createTimeSequence(dataFreq,time(2)-time(1), ...
%                               time(1)+dataOffset,0);
% exp.value = data.value(time(1)*dataFreq+1:time(2)*dataFreq+1);
% verifyEqual(testCase,act,exp,'AbsTol',1e-7);
% end
% 
% function testRandomSameFreqTimeSmall(testCase)
% load randomInterpolateTestData.mat data dataFreq dataOffset;
% refFreq = dataFreq;
% ref = 1/refFreq;
% time = [0,7];
% act = interpolate(ref, data, time);
% exp.time = createTimeSequence(dataFreq,time(2)-time(1), ...
%                               time(1)+dataOffset,0);
% exp.value = data.value(time(1)*dataFreq+1:time(2)*dataFreq+1);
% verifyEqual(testCase,act,exp,'AbsTol',1e-7);
% end
% 
% function testRandomLowMultFreqWithoutTime(testCase)
% load randomInterpolateTestData.mat data dataFreq dataDur dataOffset;
% refFreq = 50;
% ref = 1/refFreq;
% dataRefRatio = dataFreq/refFreq;
% act = interpolate(ref, data);
% tempTime = createTimeSequence(dataFreq,dataDur,dataOffset,0);
% exp.time = tempTime(1:dataRefRatio:end);
% exp.value = data.value(1:dataRefRatio:end);
% verifyEqual(testCase,act,exp,'AbsTol',1e-7);
% end
% 
% function testRandomLowMultFreqWithTime(testCase)
% load randomInterpolateTestData.mat data dataFreq dataOffset;
% refFreq = 50;
% ref = 1/refFreq;
% dataRefRatio = dataFreq/refFreq;
% time = [0,7];
% act = interpolate(ref, data, time);
% tempTime = createTimeSequence(dataFreq,time(2)-time(1), ...
%                               time(1)+dataOffset,0);
% exp.time = tempTime(1:dataRefRatio:end);
% exp.value = data.value(time(1)*dataFreq+1:dataRefRatio:time(2)*dataFreq+1);
% verifyEqual(testCase,act,exp,'AbsTol',1e-7);
% end
% 
% function testRandomLowMultFreqTimeSmall(testCase)
% load randomInterpolateTestData.mat data dataFreq dataOffset;
% refFreq = 50;
% ref = 1/refFreq;
% dataRefRatio = dataFreq/refFreq;
% time = [3,7];
% act = interpolate(ref, data, time);
% tempTime = createTimeSequence(dataFreq,time(2)-time(1), ...
%                               time(1)+dataOffset,0);
% exp.time = tempTime(1:dataRefRatio:end);
% exp.value = data.value(time(1)*dataFreq+1:dataRefRatio:time(2)*dataFreq+1);
% verifyEqual(testCase,act,exp,'AbsTol',1e-7);
% end
% 
% function testRandomHighMultFreqWithoutTime(testCase)
% load randomInterpolateTestData.mat data dataFreq dataDur dataOffset;
% refFreq = 3000;
% ref = 1/refFreq;
% refDataRatio = refFreq/dataFreq;
% act = interpolate(ref, data);
% exp.time = createTimeSequence(refFreq,dataDur,dataOffset,0);
% exp.value = simpleInterpolate(data.value,refDataRatio);
% verifyEqual(testCase,act,exp,'AbsTol',1e-6);
% end
% 
% function testRandomHighMultFreqWithTime(testCase)
% load randomInterpolateTestData.mat data dataFreq dataOffset;
% refFreq = 3000;
% ref = 1/refFreq;
% refDataRatio = refFreq/dataFreq;
% time = [0,7];
% act = interpolate(ref, data, time);
% exp.time = createTimeSequence(refFreq,time(2)-time(1), ...
%                               time(1)+dataOffset,0);
% tempValue = simpleInterpolate(data.value,refDataRatio);
% exp.value = tempValue(time(1)*refFreq+1:time(2)*refFreq+1);
% verifyEqual(testCase,act,exp,'AbsTol',1e-6);
% end
% 
% function testRandomHighMultFreqTimeSmall(testCase)
% load randomInterpolateTestData.mat data dataFreq dataOffset;
% refFreq = 3000;
% ref = 1/refFreq;
% refDataRatio = refFreq/dataFreq;
% time = [3,7];
% act = interpolate(ref, data, time);
% exp.time = createTimeSequence(refFreq,time(2)-time(1), ...
%                               time(1)+dataOffset,0);
% tempValue = simpleInterpolate(data.value,refDataRatio);
% exp.value = tempValue(time(1)*refFreq+1:time(2)*refFreq+1);
% verifyEqual(testCase,act,exp,'AbsTol',1e-6);
% end

% customInterpolateTestData.mat can be visually inspected using function 
% createCustomInterpolateTestDataMat. This shows that interpolation is done
% properly and the results can be used to test the interpolate function.
% Furthermore, this function has proven succesful for dataFreq = 1000 and
% refFreq = 951 by checking it with calculations by hand on a small time
% interval.

function testCustomLowFreqWithoutTime(testCase)
load customInterpolateTestData.mat data refFreq int;
ref = 1/refFreq;
act = interpolate(ref, data);
exp.time = int.time;
exp.value = int.value;
verifyEqual(testCase,act,exp,'AbsTol',1e-6);
end

function testCustomLowFreqWithTime(testCase)
load customInterpolateTestData.mat data dataOffset refFreq int;
timeThres = 1e-10;
ref = 1/refFreq;
time = [0,7];
act = interpolate(ref, data, time);
l = length(int.time);
for i = 1:l
    if abs(int.time(i) - (dataOffset + time(1))) < timeThres || ...
       int.time(i) > (dataOffset + time(1))
        break;
    end
end
for j = i:l
    if int.time(j) > (dataOffset + time(2))
        break;
    end
end
j = j - 1;
exp.time = int.time(i:j);
exp.value = int.value(i:j);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

function testCustomLowFreqTimeSmall(testCase)
load customInterpolateTestData.mat data dataOffset refFreq int;
timeThres = 1e-10;
ref = 1/refFreq;
time = [3,7];
act = interpolate(ref, data, time);
l = length(int.time);
for i = 1:l
    if abs(int.time(i) - (dataOffset + time(1))) < timeThres || ...
       int.time(i) > (dataOffset + time(1))
        break;
    end
end
for j = i:l
    if int.time(j) > (dataOffset + time(2))
        break;
    end
end
j = j - 1;
exp.time = int.time(i:j);
exp.value = int.value(i:j);
verifyEqual(testCase,act,exp,'AbsTol',1e-7);
end

% Use createCustomInterpolateTestDataMat.m to generate with the same and
% higher frequency

% Insert a ref signal using the same data as generated in 
% createCustomInterpolateTestDataMat.

%TODO:
%- custom test case with same, higher and lower ref freq than data.time
%(without time)
%- custom test case with same, higher and lower ref freq than data.time
%(with time 0-max, min-max)
%- custom test case with shifted variants of ref with respect to time
%(- ref as sampling time and as time sequence extra per test)


function output = createTimeSequence(sampleFreq, dur, offset, putDups)
if nargin < 4
    putDups = 0;
end

sampleTime = 1/sampleFreq;

n = sampleFreq*dur + 1;

output = zeros(1,n);
output(1) = offset;

if putDups
    randSeq = round(rand(n,1));
end

for i = 2:n
    if putDups && randSeq(i)
        output(i) = output(i-1);
    else
        output(i) = output(1) + (i-1)*sampleTime;
    end
end
end

function output = createRandomValueSequence(n, min, max)
output = min + (max - min)*rand(1,n);
end

function output = simpleInterpolate(inputArray, refDataRatio)    
l = length(inputArray);
output = zeros(1,(l-1)*refDataRatio+1);
for i = 1:l-1
    for j = 1:refDataRatio
        output(refDataRatio*(i-1)+j) = inputArray(i) + ...
            (j-1)/refDataRatio*(inputArray(i+1) - inputArray(i));
    end
end
output(end) = inputArray(end);
end