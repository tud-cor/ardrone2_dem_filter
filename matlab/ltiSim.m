function x = ltiSim(sysd,t,x0,u,param)
n = length(x0);

% Translate operating point to origin of state-input space
stateOperatingPoint = zeros(n,1);
stateOperatingPoint(3) = 1;
x0 = x0 - stateOperatingPoint;

inputOperatingPoint = [param.m*param.g; 0; 0; 0];
% inputOperatingPoint = [4.39457076058028; 0; 0; 0];
% inputOperatingPoint = [4.44; 0; 0; 0];
u = u - inputOperatingPoint;

% Simulate system on every time step
dur     = length(t);

x       = zeros(n,dur);
x(:,1)  = x0;
for i = 1:dur-1
    x(:,i+1) = sysd.A*x(:,i) + sysd.B*u(:,i);
    % Ground constraint
    if x(3,i+1) + stateOperatingPoint(3) < 0
        x(3,i+1) = -stateOperatingPoint(3);
    end
    if abs(x(3,i+1) + stateOperatingPoint(3)) < param.groundThres && ...
            x(6,i+1) < 0
        x(6,i+1) = 0;
    end
end

% Translate back to operating point in state-input space
x = x + stateOperatingPoint;
end