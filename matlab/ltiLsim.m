function [t,x] = ltiLsim(sysd,t,x0,u,param)
% Translate operating point to origin of state-input space
stateOperatingPoint = zeros(length(x0),1);
stateOperatingPoint(3) = 1;
x0 = x0 - stateOperatingPoint;

inputOperatingPoint = [param.m*param.g; 0; 0; 0];
% inputOperatingPoint = [4.39457076058028; 0; 0; 0];
% inputOperatingPoint = [4.44; 0; 0; 0];
u = u - inputOperatingPoint;

% Use lsim
x0 = x0';
u = u';
[y,t,x] = lsim(sysd,u,t,x0);
y = y';
x = x';

% Translate back to operating point in state-input space
x = x + stateOperatingPoint;
end