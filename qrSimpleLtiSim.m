function x = qrSimpleLtiSim(u,t,x0,param)
% Translate operating point to origin of state-input space
n = length(x0);
l = size(u,1);
stateOperatingPoint = zeros(n,1);
stateOperatingPoint(3) = 1;
inputOperatingPoint = [param.m*param.g; 0; 0; 0];
x0 = x0 - stateOperatingPoint;
u = u - inputOperatingPoint;

% Construct continuous-time linearised state space system
A = [0, 0, 0, 1, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 1, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , param.g, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, -param.g, 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 1, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0];
B = [0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     1/param.m, 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 1/param.ixx, 0          , 0;
     0        , 0          , 1/param.iyy, 0;
     0        , 0          , 0          , 1/param.izz];
C = eye(n);
D = zeros(n,l);

sysc = ss(A,B,C,D);

% Construct discrete-time linearised state space system
sysd = c2d(sysc,param.sampleTime);

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

% Use lsim
% x0 = x0';
% u = u';
% [y2,t2,x2] = lsim(sysd,u,t,x0);
% u = u';
% y2 = y2';
% x2 = x2';

% State comparison for-loop vs lsim
% x = x + stateOperatingPoint;
% x2 = x2 + stateOperatingPoint;
% figure('Name','State comparison');
% subplot(2,2,1);
% plot(t,x(2,:));
% hold on;
% plot(t2,x2(2,:));
% xlabel('Time (s)');
% ylabel('y (m)');
% legend('Original','lsim');
% subplot(2,2,2);
% plot(t,x(5,:));
% hold on;
% plot(t2,x2(5,:));
% xlabel('Time (s)');
% ylabel('yDot (m/s)');
% legend('Original','lsim');
% subplot(2,2,3);
% plot(t,x(7,:));
% hold on;
% plot(t2,x2(7,:));
% xlabel('Time (s)');
% ylabel('\phi (m)');
% legend('Original','lsim');
% subplot(2,2,4);
% plot(t,x(10,:));
% hold on;
% plot(t2,x2(10,:));
% xlabel('Time (s)');
% ylabel('phiDot (m)');
% legend('Original','lsim');
% 
% % Plot inputs
% figure('Name','Linearised inputs');
% subplot(4,1,1);
% plot(u(1,:));
% xlabel('Time (s)');
% ylabel('T (N)');
% subplot(4,1,2);
% plot(u(2,:));
% xlabel('Time (s)');
% ylabel('\tau_\phi (N)');
% subplot(4,1,3);
% plot(u(3,:));
% xlabel('Time (s)');
% ylabel('\tau_\theta (N)');
% subplot(4,1,4);
% plot(u(4,:));
% xlabel('Time (s)');
% ylabel('\tau_\psi (N)');

% Translate back to operating point in state-input space
x = x + stateOperatingPoint;
end