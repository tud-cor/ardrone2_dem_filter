%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate system data in generalized coordinates
%
% Function to generate all system matrices, noise precision matrices and,
% if indicated, synthetic data, in generalized coordinates.
% 
% Code source:     https://github.com/ajitham123/DEM_observer
% Original author: Ajith Anil Meera, TU Delft, CoR
% Adjusted by:     Dennis Benders, TU Delft, CoR
% Last modified:   09.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [model,brain] = generative_process(model,brain,if_dataset)

% Create necessary system matrices to be used in the filters
Tt  = 0:model.sam_time:model.t_end-model.sam_time;

brain.At = kron(eye(brain.p+1),model.A);
brain.Bt = kron(eye(brain.p+1,brain.d+1),model.B);
brain.Ct = kron(eye(brain.p+1),model.C);

T = toeplitz(zeros(1,brain.p+1),[0 1 zeros(1,brain.p-1)]);
if brain.p==0
    T=0;
end
brain.Da = kron(T,eye(size(model.A,2)));

T = toeplitz(zeros(1,brain.d+1),[0 1 zeros(1,brain.d-1)]);
if brain.d==0
    T=0;
end
brain.Dv = kron(T,eye(brain.nv));

brain.D_A = brain.Da-brain.At;

% Construct precision matrixes of process, measurement and input noise
if exist('model') && isfield(model,'Pw')
    brain.Pw = model.Pw;
else
    model.Pw = eye(brain.nx)/model.sigma_w^2;
    brain.Pw = model.Pw;
end
if exist('model') && isfield(model,'Pz')
    brain.Pz = model.Pz;
else
    model.Pz = eye(brain.ny)/model.sigma_z^2;
    brain.Pz = model.Pz;
end

[brain.W0,brain.V0y,brain.V0v] = ...
    precision_matrix(brain.s,brain.Pw,brain.Pz,brain.sigma_v,brain.p,...
                     brain.d,brain.nv,brain.nx,brain.ny);

% Uncomment to construct precision matrices using s = 0
% brain.W0 = zeros((brain.p+1)*brain.nx);
% brain.W0(1:brain.nx,1:brain.nx) = model.Pw;
% brain.V0y = zeros((brain.p+1)*brain.ny);
% brain.V0y(1:brain.ny,1:brain.ny) = model.Pz;
% brain.V0v = zeros((brain.d+1)*brain.nv);
% brain.V0v(1:brain.nv,1:brain.nv) = 1/brain.sigma_v^2;

brain.V0 = blkdiag(brain.V0y,brain.V0v);

% Generate data if no dataset is used
if if_dataset == 0
    % Generate noise
    [noise_w, noise_z] = make_noise(model.s,model.Pw,model.Pz,Tt,...
        model.sam_time,brain.nx,brain.ny,brain.nt);
    brain.noise_w = noise_w;
    brain.noise_z = noise_z;

    % Create system model
    process = ss(model.A,...
                 [model.B,eye(brain.nx),zeros(brain.nx,brain.ny)],...
                 model.C,...
                 [zeros(brain.ny,1),zeros(brain.ny,brain.nx),...
                  eye(brain.ny)]);
    process = c2d(process,model.sam_time,'zoh');

    % Generate synthetic data
    [process_x,process_y] = generate_data(model.A,model.B,model.C,...
                                          noise_w,noise_z,...
                                          model.real_cause,model.t,...
                                          model.sam_time,model.p);

    model.ideal_x = process_x;
    model.process_x = process_x;
    model.process_y = process_y;
end

end
