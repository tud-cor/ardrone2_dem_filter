function [xExp,xExpSimpleDer] = getFullState(expData)
% Dimension initialization
n = 12;
dur = length(expData.output.otTime);

% Output initialization
xExp = zeros(n,dur);
xExpSimpleDer = zeros(n,dur);

% Matrix for finite differences initialization
dt = 0.01;
p = 1;
o = 6;
dim = 3;
method = 'c';
E = f_finitediffmat(dt,p,o,dim,method);
E = E(4:6,:);

for i = 1:dur
    % Position
    xExp(1:3,i) = expData.output.otPos(:,i);

    % Find central time to use for central differences approach
    [~,cIdx] = min(abs(expData.output.highFreq.otTime-...
                       expData.output.otTime(i)));
    sIdx = cIdx - 0.5*(p+o-1);
    eIdx = cIdx + 0.5*(p+o-1);

    % Linear velocity (by taking derivatives using finite differences)
    xExp(4:6,i) = E*reshape(expData.output.highFreq.otPos(:,sIdx:eIdx),...
                            [(p+o)*dim,1]);

    % Orientation
    xExp(7:9,i) = expData.output.otOrient(:,i);

    % Angular velocity (by taking derivatives using finite differences)
    xExp(10:12,i) = E*...
        reshape(expData.output.highFreq.otOrient(:,sIdx:eIdx),...
                [(p+o)*dim,1]);
end

for i = 1:dur
    % Position
    xExpSimpleDer(1:3,i) = expData.output.otPos(:,i);

    % Linear velocity (by taking simple derivatives)
%     [~,cIdx] = min(abs(expData.output.highFreq.otTime-...
%                        expData.output.otTime(i)));
%     xExpSimpleDer(4:6,i) = (expData.output.highFreq.otPos(:,cIdx) - ...
%                             expData.output.highFreq.otPos(:,cIdx-1))/...
%                            expData.sampleTimeHighFreq;
    if i == dur
        xExpSimpleDer(4:6,i) = xExpSimpleDer(4:6,i-1);
    else
        xExpSimpleDer(4:6,i) = (expData.output.otPos(:,i+1) - ...
                                expData.output.otPos(:,i))/...
                               expData.sampleTime;
    end

    % Orientation
    xExpSimpleDer(7:9,i) = expData.output.otOrient(:,i);

    % Angular velocity (by taking simple derivatives)
%     xExpSimpleDer(10:12,i) = ...
%         (expData.output.highFreq.otOrient(:,cIdx) - ...
%          expData.output.highFreq.otOrient(:,cIdx-1))/...
%         expData.sampleTimeHighFreq;
    if i == dur
        xExpSimpleDer(10:12,i) = ...
        xExpSimpleDer(10:12,i-1);
    else
        xExpSimpleDer(10:12,i) = ...
        (expData.output.otOrient(:,i+1) - ...
         expData.output.otOrient(:,i))/...
        expData.sampleTime;
    end
end
end