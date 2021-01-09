%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output derivative generation
%
% Function to obtain the derivatives of outputs in order to convert the
% generative process to generalized coordinates.
% 
% Code source:     https://github.com/ajitham123/DEM_observer
% Original author: Ajith Anil Meera, TU Delft, CoR
% Adjusted by:     Dennis Benders, TU Delft, CoR
% Last modified:   09.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Y_embed = ...
         generalized_process(process_y,prior_cause,t,sam_time,nv,ny,p,d)

% Determine dimensions
nt = size(t,2);
Y_embed = zeros(ny*(p+1)+nv*(d+1),nt);

% Beginning and end of signals cannot be used to construct proper
% derivatives -> assume derivatives of 0 at beginning and end
for i = 1:nt
    if i>p+1 && i<nt-p-1   % Use this section for low sampling time
        Y_embed(:,i) = [embed_Y(process_y',p+1,t(i),sam_time);...
                        -embed_Y(prior_cause,d+1,t(i),sam_time)];
    else
        Y_embed(1:ny,i) = process_y(i,:)';
        Y_embed(ny*(p+1)+1:ny*(p+1)+nv,i) = -prior_cause(:,i);
    end
end

end
