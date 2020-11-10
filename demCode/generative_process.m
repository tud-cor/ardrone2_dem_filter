function [model,brain] = generative_process(model,brain,if_dataset)

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

brain.Pz = model.Pz;
brain.Pw = model.Pw;

% EDIT Precision of process noises, measurement noise, input noise
[brain.W0,brain.V0y,brain.V0v] = precision_matrix(brain.s,brain.Pw,...
                 brain.Pz,brain.sigma_v,brain.p,brain.d,brain.nv,brain.nx,brain.ny);
brain.V0 = blkdiag(brain.V0y,brain.V0v);

if if_dataset==0
    % generate noise
    [noise_w, noise_z] = make_noise(model.s,model.Pw,model.Pz,Tt,...
        model.sam_time,brain.nx,brain.ny,brain.nt);
    brain.noise_w = noise_w;
    brain.noise_z = noise_z;

    % generative process
    process = ss(model.A,...
                 [model.B,eye(brain.nx),zeros(brain.nx,brain.ny)],...
                 model.C,...
                 [zeros(brain.ny,1),zeros(brain.ny,brain.nx),...
                  eye(brain.ny)]);
    process = c2d(process,model.sam_time,'zoh');

    [process_x,process_y] = generate_data(model.A,model.B,model.C,...
                                          noise_w,noise_z,...
                                          model.real_cause,model.t,...
                                          model.sam_time,model.p);
    model.ideal_x = process_x;
    model.process_x = process_x;
    model.process_y = process_y;
end
end