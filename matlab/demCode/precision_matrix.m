%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Precision matrix construction
%
% Function to create the process, measurement and input precision matrices.
% 
% Code source:     https://github.com/ajitham123/DEM_observer
% Original author: Ajith Anil Meera, TU Delft, CoR
% Adjusted by:     Dennis Benders, TU Delft, CoR
% Last modified:   09.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [W0,V0y,V0v] = precision_matrix(s_brain,P_brain_w,P_brain_z,...
                                         sigma_v,p,d,nv,nx,ny)

% Determine maximum embedding order
if p>d
    pp = p;
else
    pp = d;
end

% Create the temporal correlation matrix (eq. 53 in DEM paper)
k          = 0:pp;
x          = sqrt(2)*s_brain;
r(1 + 2*k) = cumprod(1 - 2*k)./(x.^(2*k));

Cov     = [];
for i = 1:pp+1
    Cov = [Cov; r([1:pp+1] + i - 1)];
    r = -r;
end

Cov_inv = inv(Cov);

% Create process, measurement and input precision matrices
W0 = kron(Cov_inv(1:p+1,1:p+1),P_brain_w);
V0y = kron(Cov_inv(1:p+1,1:p+1),P_brain_z);
V0v = kron(Cov_inv(1:d+1,1:d+1),eye(nv)/sigma_v^2);

end
