%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate synthetic data
%
% Function to generate synthetic data, based on system and noise properties
% given in DEM_observer.m.
% 
% Code source:     https://github.com/ajitham123/DEM_observer
% Original author: Ajith Anil Meera, TU Delft, CoR
% Adjusted by:     Dennis Benders, TU Delft, CoR
% Last modified:   09.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [process_x, process_y] = generate_data(A,B,C,noise_w,noise_z,...
                                                real_cause,t,sam_time,p)

% Choose whether generalized coordinates should be used when generating
% data
% 0: do not use generalized coordinates
% 1: use generalized coordinates
if_GC = 0;

% Determine dimensions
nx = size(A,1);
ny = size(C,1);
nt = size(t,2);

% Initialize arrays
process_x = zeros(nx,nt)';
process_y = zeros(ny,nt)';
process_y(1,:) = noise_z(:,1);
U.x = zeros(nx,p+1);
U.v = zeros(ny+1,p+1);

% Create system state-space
if if_GC == 0
    ideal_sp = c2d(ss(A,B,C,zeros(ny,1)),sam_time,'zoh');
end

% Generate data
if if_GC
    for i=1:nt-1
        d_noise_w = embed_Y(noise_w,p+1,t(i),sam_time);
        d_noise_z = embed_Y([noise_z; real_cause],p+1,t(i),sam_time);
        d_noise_w = reshape(d_noise_w,nx,[]);
        d_noise_z = reshape(d_noise_z,ny+1,[]);

        pU.x(:,1)  = U.x(:,1);
        pU.v(:,1)  = [C*U.x(:,1); 0] + [noise_z(:,i); real_cause(1,i)];
        pU.x(:,2)  = A*U.x(:,1) + [zeros(size(C')) B]*U.v(:,1) + ...
                     noise_w(:,i);
        for j = 2:p
            pU.v(:,j) = [C*pU.x(:,j); 0] + d_noise_z(:,j);
            pU.x(:,j+1) = [zeros(size(C')) B]*pU.v(:,j) + A*pU.x(:,j) + ...
                d_noise_w(:,j);
        end
        pU.v(:,p+1) = [C*pU.x(:,p+1); 0] + d_noise_z(:,p+1);

        Dv_j = kron(diag(ones(1,p),1),eye(ny+1));
        Dx_j = kron(diag(ones(1,p),1),eye(nx));

        dgdv = zeros((p+1)*(ny+1));
        dgdx = kron(diag(ones(1,p),1),[C; zeros(1,nx)]);
        dfdv = kron(eye(p+1),[zeros(size(C')) B]);
        dfdx = kron(eye(p+1),A);
        dfdw = kron(eye(p+1),eye(nx));
        J = [dgdv dgdx Dv_j zeros(size(Dv_j,1),size(Dx_j,2));...
            dfdv dfdx zeros((p+1)*nx,size(Dv_j,2)) dfdw;...
            zeros((p+1)*[(ny+1),(ny+1+nx)]) Dv_j ...
            zeros(size(Dv_j,1),size(Dx_j,2));...
            zeros((p+1)*[nx,(ny+1+nx)]) ...
            zeros(size(Dx_j,1),size(Dv_j,2)) Dx_j];
        qU = [reshape(pU.v,[],1); reshape(pU.x,[],1); ...
            reshape(d_noise_z,[],1); reshape(d_noise_w,[],1)];
        D_j = blkdiag(Dv_j,Dx_j,Dv_j,Dx_j);
        qU = qU + (expm(J*sam_time) - eye(size(J)))*pinv(J)*D_j*qU;

        U.v = reshape(qU(1:(p+1)*(ny+1),1),ny+1,p+1);
        U.x = reshape(qU((p+1)*(ny+1)+1:(p+1)*(ny+1+nx),1),...
            nx,p+1);
        process_y(i+1,:) = U.v(1:ny,1);
        process_x(i+1,:) = U.x(:,1);
    end
else
    for i=2:nt
        process_x(i,:) = (ideal_sp.A*process_x(i-1,:)' + ...
            ideal_sp.B*real_cause(1,i-1)+ noise_w(:,i-1))';
        process_y(i-1,:) = (ideal_sp.C*process_x(i-1,:)' + ...
            ideal_sp.D*real_cause(1,i-1)+ noise_z(:,i-1))';
    end
    process_y(end,:) = (ideal_sp.C*process_x(end,:)' + ...
        ideal_sp.D*real_cause(1,end)+ noise_z(:,end))';
end

end
