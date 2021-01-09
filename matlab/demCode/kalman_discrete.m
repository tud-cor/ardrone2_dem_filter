%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kalman filter
%
% Function to obtain Kalman filter state estimates, based on the
% conventional scheme: https://en.wikipedia.org/wiki/Kalman_filter.
% 
% Code source:     https://github.com/ajitham123/DEM_observer
% Original author: Ajith Anil Meera, TU Delft, CoR
% Adjusted by:     Dennis Benders, TU Delft, CoR
% Last modified:   09.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function x = kalman_discrete(A,B,C,process_y,P_noise_z,P_noise_w,...
                             sam_time,t,real_cause,if_causes)

% Create system matrices
% Bd = (expm(A*sam_time)-eye(size(A)))*pinv(A)*B;
sysC = ss(A,B,C,[]);
sysD = c2d(sysC,sam_time);
Ad = sysD.A;
Bd = sysD.B;
Cd = sysD.C;

% Determine dimensions
nx = size(A,1);
nt = size(t,2);

% Create noise covariance matrices
x_prev = zeros(nx,1);
Q = inv(P_noise_w);
R = inv(P_noise_z);
x  = zeros(nx,nt);           % EKF estimate of the mean of states
% Q  = Q + expm(A)*B*exp(0)*B'*expm(A)'; % EKF process noise variance

% Create state estimate covariance matrix
% P = {pinv(full(C'*R*C))}; % EKF conditional covariance of states
% P = {zeros(nx)};
P = {eye(2)*1e6};

% Obtain Kalman filter state estimates
for i = 2:nt

    % Prediction step
    xPred    = Ad*x_prev + if_causes*Bd*real_cause(:,i);
    Jx       = Ad;
    PPred    = Q + Jx*P{i-1}*Jx';

    % Update step
    yPred    = Cd*xPred;
    Jy       = Cd;
    K        = PPred*Jy'*inv(R + Jy*PPred*Jy');
    x_prev   = xPred + K*(process_y(i,:)' - yPred);
    x(:,i)   = x_prev;
    P{i}     = PPred - K*Jy*PPred;

end

end
