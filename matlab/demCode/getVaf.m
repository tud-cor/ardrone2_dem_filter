%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% VAF error function
%
% Function to obtain the VAF values of state estimates.
% 
% Author:          Dennis Benders, TU Delft, CoR
% Last modified:   09.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vaf = getVaf(x,ref)

% Determine amount of samples
n = size(x,1);

% Calculate MSE
se = zeros(n,1);
for i = 1:n
    se(i) = norm(ref(i,:) - x(i,:))^2;
end
mse = mean(se);

% Calculate MSN of ref
snRef = zeros(n,1);
for i = 1:n
    snRef(i) = norm(ref(i,:))^2;
end
msnRef = mean(snRef);

% Calculate VAF
vaf = max(0,(1 - mse/msnRef)*100);

end
