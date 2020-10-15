function x = ltiStepSim(sysd,t,xExp,uExp,op)
% Dimensions initialization
n = 12;
dur = length(t);

% States and inputs initialization
x = zeros(n,dur);
x(:,1) = xExp(:,1);


% Shift operating point to origin of state-input space
xExpOP = xExp - op.x;
uExpOP = uExp - op.u;

% Calculate next state using the current state and input
for i = 1:dur-1
    x(:,i+1) = sysd.A*xExpOP(:,i) + sysd.B*uExpOP(:,i);
end

% Shift state back to operating point
x(:,2:end) = x(:,2:end) + op.x;
end