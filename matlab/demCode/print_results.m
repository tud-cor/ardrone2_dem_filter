%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Result printing
%
% Function to print the observer (hidden) state and input estimates
% results.
% 
% Code source:     https://github.com/ajitham123/DEM_observer
% Original author: Ajith Anil Meera, TU Delft, CoR
% Adjusted by:     Dennis Benders, TU Delft, CoR
% Last modified:   20.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function print_results(SSE,if_cause,xh)

% Print estimation results with hidden state
if xh
    if if_cause
        fprintf('\nSum of Squared Errors (SSE) - known input\n');

        fprintf('\n');

        fprintf('State  |     Hidden | Observable |        All\n');
        fprintf('---------------------------------------------\n');
        fprintf('DEM    | %10.4f | %10.4f | %10.4f\n',...
                SSE.DEMv.xh,SSE.DEMv.xobs,SSE.DEMv.x);
        fprintf('Kalman | %10.4f | %10.4f | %10.4f\n',...
                SSE.kalmanv.xh,SSE.kalmanv.xobs,SSE.kalmanv.x);

        fprintf('\n');
    else
        fprintf('\nSum of Squared Errors (SSE) - unknown input\n');

        fprintf('\n');

        fprintf('State  |     Hidden | Observable |        All\n');
        fprintf('---------------------------------------------\n');
        fprintf('DEM    | %10.4f | %10.4f | %10.4f\n',...
                SSE.DEM.xh,SSE.DEM.xobs,SSE.DEM.x);
        fprintf('Kalman | %10.4f | %10.4f | %10.4f\n',...
                SSE.kalman.xh,SSE.kalman.xobs,SSE.kalman.x);

        fprintf('\n');

        fprintf('Input  |        All\n');
        fprintf('-------------------\n');
        fprintf('DEM    | %10.4f \n',SSE.DEM.v);

        fprintf('\n');
    end


% Print estimation results without hidden state
else
    if if_cause
        fprintf('\nSum of Squared Errors (SSE) - known input\n');

        fprintf('\n');

        fprintf('State  |        All\n');
        fprintf('-------------------\n');
        fprintf('DEM    | %10.4f\n',SSE.DEMv.x);
        fprintf('Kalman | %10.4f\n',SSE.kalmanv.x);

        fprintf('\n');
    else
        fprintf('\nSum of Squared Errors (SSE) - unknown input\n');

        fprintf('\n');

        fprintf('State  |        All\n');
        fprintf('-------------------\n');
        fprintf('DEM    | %10.4f\n',SSE.DEM.x);
        fprintf('Kalman | %10.4f\n',SSE.kalman.x);

        fprintf('\n');

        fprintf('Input  |        All\n');
        fprintf('-------------------\n');
        fprintf('DEM    | %10.4f \n',SSE.DEM.v);

        fprintf('\n');
    end
end

end
