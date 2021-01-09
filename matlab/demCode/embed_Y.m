%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Derivative generation
%
% Function to obtain the derivatives of inputs/outputs using an inverse
% Taylor expansion with central approach.
% 
% Code source:     https://github.com/ajitham123/DEM_observer
% Original author: Ajith Anil Meera, TU Delft, CoR
% Adjusted by:     Dennis Benders, TU Delft, CoR
% Last modified:   09.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dY = embed_Y(Y,n,t,dt)

% Index initialization
[~, N]  = size(Y);
T = zeros(n);

s      = round((t)/dt);
k      = (1:n)  + fix(s - (n + 1)/2);
x      = s - min(k) + 1;

% Extra step: truncate innacurate derivative at edges
y_pad = k<x-1 | k> N-(x-1);

i      = k < 1;
k      = k.*~i + i;
i      = k > N;
k      = k.*~i + i*N;


% Inverse embedding operator (T): cf, Taylor expansion Y(t) <- T*y[:]
for i = 1:n
    for j = 1:n
        T(i,j) = ((i - x)*dt)^(j - 1)/prod(1:(j - 1));
    end
end

% Embedding operator: y[:] <- E*Y(t)
E     = inv(T);

% Obtain derivatives and truncate inaccuracte derivatives
dY      = Y(:,k)*E';
dY(:,end-sum(y_pad)+1:end) = zeros(size(Y,1),sum(y_pad));
dY = reshape(dY,[],1);

end
