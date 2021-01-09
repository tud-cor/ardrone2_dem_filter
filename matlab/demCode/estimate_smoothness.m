%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Smoothness estimation
%
% Function to obtain a smoothness estimate properly representing the system
% noises using particle swarm optimization.
% 
% Code source:     https://github.com/ajitham123/DEM_observer
% Original author: Ajith Anil Meera, TU Delft, CoR
% Adjusted by:     Dennis Benders, TU Delft, CoR
% Last modified:   09.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function s_est = estimate_smoothness(noise_z,Tt)

% Define objective function
func = @(k) find_s(k,noise_z,Tt);

% Set optimization options
options = optimoptions('particleswarm','SwarmSize',50,...
            'HybridFcn',@fmincon,'MaxIterations',4,'Display','iter');

% Optimize
s_est = particleswarm(func,1,0.01,1,options)

end
