function xExp = getFullState(expData)
% Dimension initialization
n = 12;
dur = length(expData.state.otTime);

% Output initialization
xExp = zeros(n,dur);

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
    xExp(1:3,i) = expData.state.otPos(:,i);

    % Find central time to use for central differences approach
    [~,cIdx] = min(abs(expData.state.highFreq.otTime-...
                       expData.state.otTime(i)));
    sIdx = cIdx - 0.5*(p+o-1);
    eIdx = cIdx + 0.5*(p+o-1);

    % Linear velocity (by taking derivatives using finite differences)
    xExp(4:6,i) = E*reshape(expData.state.highFreq.otPos(:,sIdx:eIdx),...
                            [(p+o)*dim,1]);

    % Orientation
    xExp(7:9,i) = expData.state.otOrient(:,i);

    % Angular velocity (by taking derivatives using finite differences)
    xExp(10:12,i) = E*...
        reshape(expData.state.highFreq.otOrient(:,sIdx:eIdx),...
                [(p+o)*dim,1]);
end
end