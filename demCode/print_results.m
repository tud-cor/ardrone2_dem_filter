function print_results(SSE,if_UIO,if_cause,xh)

% if if_UIO == 1
%     fprintf('DEM_x, UIO_x, KF_x = ');
%     disp(vpa([SSE.DEM.x(j,k) SSE.UIO.x(j,k) SSE.kalman.x(j,k)],4))
%     if if_cause == 1
%         fprintf('DEMv_x, KFv_x = ');
%         disp(vpa([SSE.DEMv.x(j,k) SSE.kalmanv.x(j,k)],4))
%     end
%     fprintf('DEM_v, UIO_v = ');
%     disp(vpa([SSE.DEM.v(j,k) SSE.UIO.v(j,k)],4));
% else
%     fprintf('DEM_x, KF_x, DEM_v = ');
%     disp(vpa([SSE.DEM.x(j,k) SSE.kalman.x(j,k) SSE.DEM.v(j,k)],4))
%     if if_cause == 1
%         fprintf('DEMv_x, KFv_x = ');
%         disp(vpa([SSE.DEMv.x(j,k) SSE.kalmanv.x(j,k)],4))
%     end
% end

if xh
    if if_cause
        fprintf('Sum of Squared Errors (SSE) - known input\n');
        fprintf('DEM all states   : %9.4f        DEM hidden state(s)   : %9.4f\n',SSE.DEMv.x,SSE.DEMv.xh);
        fprintf('Kalman all states: %9.4f        Kalman hidden state(s): %9.4f\n',SSE.kalmanv.x,SSE.kalmanv.xh);

        fprintf('\n');
    end

%     fprintf('Sum of Squared Errors (SSE) - unknown input\n');
%     fprintf('DEM all states   : %9.4f        DEM hidden state(s)   : %9.4f        DEM input: %9.4f\n',SSE.DEM.x,SSE.DEM.xh,SSE.DEM.v);
%     fprintf('Kalman all states: %9.4f        Kalman hidden state(s): %9.4f\n',SSE.kalman.x,SSE.kalman.xh);
else
    if if_cause
        fprintf('Sum of Squared Errors (SSE) - known input\n');
        fprintf('DEM all states   : %9.4f\n',SSE.DEMv.x);
        fprintf('Kalman all states: %9.4f\n',SSE.kalmanv.x);

        fprintf('\n');
    end

%     fprintf('Sum of Squared Errors (SSE) - unknown input\n');
%     fprintf('DEM all states   : %9.4f        DEM input: %9.4f\n',SSE.DEM.x,SSE.DEM.v);
%     fprintf('Kalman all states: %9.4f\n',SSE.kalman.x);
end
end