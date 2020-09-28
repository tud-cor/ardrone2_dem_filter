function [x,mse] = ltiStepSim(sysd,t,xExp,uExp,param)
% The 2-norm (||x||_2) is used as error metric.

% Dimensions initialization
n = 12;
dur = length(t);

% States and inputs initialization
x = zeros(n,dur);
x(:,1) = xExp(:,1);

% Operating point initalization
stateOperatingPoint = zeros(n,1);
stateOperatingPoint(3) = 1;
inputOperatingPoint = [param.m*param.g;0;0;0];

% Error initialization
se = zeros(1,dur);
se(1) = 0;

for i = 1:dur-1
    % Shift operating point to origin of state-input space
    xExp(:,i) = xExp(:,i) - stateOperatingPoint;
    uExp(:,i) = uExp(:,i) - inputOperatingPoint;

    % Calculate next state using the current state and input
    x(:,i+1) = sysd.A*xExp(:,i) + sysd.B*uExp(:,i);

    % Shift state back to operating point
    x(:,i+1) = x(:,i+1) + stateOperatingPoint;

    % Calculate the Squared Error (SE) by comparing next state with next
    % state derived from OptiTrack data
    se(i+1) = norm(x(:,i+1) - xExp(:,i+1))^2;
end

% Valid data in se spans 2:end (for-loop goes from 2 to end time)
se = se(2:end);

% Calculate MSE
mse = mean(se);
end