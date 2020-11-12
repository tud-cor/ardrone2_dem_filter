%% Initialization
clear;
close all;
clc;

% Choose whether inputs should also be estimated or not
% 0: inputs should be estimated
% 1: inputs are known
if_cause = 1;

% Set the index of the hidden state that is compared with an external
% reference
% 0 means no hidden state
xh = 2;

% Necessary intitialization without meaning when comparing DEM and Kalman,
% based on drone flight data
if_dataset = 1; %to generate data: set model.s=0.5; model.sigma_z=exp(-4)
if_predict_y = 0;
if_UIO = 0;
UIO_gamma = 125;
UIO_gains = [1200 1200 1200 1400 1500];


%% Test filter results for ranges in p, d and s values
% s_range = [0.001:0.001:0.015];
% p_range = 0:1:6;
% d_range = 0:1:6;

ts = 1/120;
s_range = [1e-3:1e-3:0.02];
p_range = 1;
d_range = 1;

n_p = length(p_range);
n_d = length(d_range);
n_s = length(s_range);

SSE.DEMv_x = zeros(n_p,n_d,n_s);
SSE.kalmanv_x = zeros(n_p,n_d,n_s);
SSE.DEMv_xh = zeros(n_p,n_d,n_s);
SSE.kalmanv_xh = zeros(n_p,n_d,n_s);

% for i = 1:n_p
%     for j = 1:n_d
%         for k = 1:n_s
%% Get model and corresponding flight data
% Tt                Time vector (starting from 0)
% model.real_cause  System input (measured)
% model.process_x   States state (only used for plotting)
% model.process_y   System output (measured)
% model.sam_time    Sampling time
% model.A           A matrix
% model.B           B matrix
% model.C           C matrix
if if_dataset
    [Tt,model.sam_time,model.real_cause,model.process_x,model.process_y,...
     model.Pz,model.Pw,model.s,model.A,model.B,model.C] = ardrone2_flight_data;

    model.ideal_x = model.process_x;
    model.t       = Tt + model.sam_time;
    model.t_end   = model.t(end);
else
    model.sam_time = .1;
    model.t_end = 32;
    Tt  = 0:model.sam_time:model.t_end-model.sam_time;
    model.t = Tt + model.sam_time;

    model.real_cause = exp(-.25*(model.t-12).^2);

    model.A = [-.25 1; -.5 -.25];
    model.B = [1; 0];
    model.C = [.125 .1633; .125 .0676; .125 -.0676; .125 -.1633];
end

brain.nv = size(model.B,2);
brain.ny = size(model.C,1);
brain.nx = size(model.A,1);
brain.nt = size(model.t,2);


%% Set noise properties
% TODO Kernel width - probably tune
% model.s is estimated from data; orig:0.5
% model.s = s_range(k);
brain.s = model.s;

% TODO Embedding orders - probably tune
% Dataset, so model.p and model.d can be ignored
model.p = 6; %embedding order states in model
model.d = 2; %embedding order inputs in model
% brain.p = p_range(i); %embedding order states; orig:6
% brain.d = d_range(j); %embedding order inputs; orig:2
brain.p = 2; %embedding order states; orig:6
brain.d = 5; %embedding order inputs; orig:2

% TODO Standard deviations - probably tune
% Pz and Pw are defined in generative_process.m
% model.sigma_z is estimated from data; orig:exp(-4)
% model.sigma_w = model.sigma_z; %orig:exp(-4)
% model.sigma_w = 0.001;
% model.sigma_z = 0.1;
brain.sigma_v = 10; %prior of standard deviation of input; orig:exp(4)
% brain.sigma_w = model.sigma_w;
% brain.sigma_z = model.sigma_z;

% Input prior
model.prior_cause = 0.01*ones(brain.nv,brain.nt); %orig:0.5*...


%% Generate generalized state and precision matrices
[model,brain] = generative_process(model,brain,if_dataset);


%% Generate generalized coordinates for output and input (input for filter)
brain.Y_embed = generalized_process(model.process_y,model.prior_cause,...
                                    model.t,model.sam_time,...
                                    brain.nv,brain.ny,brain.p,brain.d);


%% Run filters
output = observer(model,brain,if_UIO,UIO_gamma,if_cause,if_dataset,...
                  if_predict_y);


%% Get estimation error
% t_trim:
% 1: trim out estimates at both ends (inaccurate derivatives)
% 2: trim out estimates only at end
if_t_trim = 2;
if if_t_trim == 1
    t_trim = brain.p+2:brain.nt-brain.p-2;
elseif if_t_trim == 2
    t_trim = 1:brain.nt-brain.p-2;
else
    t_trim = 1:brain.nt;
end

if if_cause == 1
    SSE.DEMv.x	  = sum(sum((output.DEMv_x(t_trim,1:brain.nx)-...
                             model.ideal_x(t_trim,:)).^2));
    SSE.kalmanv.x = sum(sum((output.kalmfv_x(:,t_trim)'-...
                             model.ideal_x(t_trim,:)).^2));
    if xh
        SSE.DEMv.xh	   = sum(sum((output.DEMv_x(t_trim,xh)-...
                                  model.ideal_x(t_trim,xh)).^2));
        SSE.kalmanv.xh = sum(sum((output.kalmfv_x(xh,t_trim)'-...
                                  model.ideal_x(t_trim,xh)).^2));
        xobs = 1:brain.nx;
        xobs(xh) = [];
        SSE.DEMv.xobs    = sum(sum((output.DEMv_x(t_trim,xobs)-...
                                    model.ideal_x(t_trim,xobs)).^2));
        SSE.kalmanv.xobs = sum(sum((output.kalmfv_x(xobs,t_trim)'-...
                                    model.ideal_x(t_trim,xobs)).^2));
    end
else
    SSE.DEM.x	 = sum(sum((output.DEM_x(t_trim,1:brain.nx)-...
                        model.ideal_x(t_trim,:)).^2));
    SSE.kalman.x = sum(sum((output.kalman_x(:,t_trim)'-...
                            model.ideal_x(t_trim,:)).^2));
    SSE.DEM.v    = sum(sum((output.DEM_x(t_trim,brain.nx*(brain.p+1)+1)-...
                            model.real_cause(:,t_trim)').^2));
    if xh
        SSE.DEM.xh	  = sum(sum((output.DEM_x(t_trim,xh)-...
                                 model.ideal_x(t_trim,xh)).^2));
        SSE.kalman.xh = sum(sum((output.kalman_x(xh,t_trim)'-...
                                 model.ideal_x(t_trim,xh)).^2));
    end
end

% if if_UIO == 1
%     SSE.UIO.x = sum(sum((output.UIO_x_est(:,t_trim)-...
%                          model.ideal_x(t_trim,:)').^2));
%     SSE.UIO.v = sum((output.UIO_v_est(:,t_trim)-...
%                      model.real_cause(:,t_trim)).^2);
% end

% SSE_mean_x(i,:) = mean([SSE.kalman.x' SSE.UIO.x' SSE.DEM.x' ...
%                         SSE.kalmanv.x' SSE.DEMv.x']);
% SSE_std_x(i,:)  = std( [SSE.kalman.x' SSE.UIO.x' SSE.DEM.x' ...
%                         SSE.kalmanv.x' SSE.DEMv.x']);
% SSE_mean_v(i,:) = mean([SSE.UIO.v' SSE.DEM.v']);
% SSE_std_v(i,:)  = std( [SSE.UIO.v' SSE.DEM.v']);
% 
% SSE_mean_x(i,:) = mean([SSE.kalman.x(1,:); SSE.DEM.x],2)';
% SSE_std_x(i,:)  = std([SSE.kalman.x(1,:); SSE.DEM.x],0,2)';
% SSE_mean_v(i,:) = mean(SSE.DEM.v,2)';
% SSE_std_v(i,:)  = std(SSE.DEM.v,0,2)';


% SSE.DEMv_x(i,j,k) = sum(sum((output.DEMv_x(t_trim,1:brain.nx)-...
%                              model.ideal_x(t_trim,:)).^2));
% SSE.kalmanv_x(i,j,k) = sum(sum((output.kalmfv_x(:,t_trim)'-...
%                                 model.ideal_x(t_trim,:)).^2));
% SSE.DEMv_xh(i,j,k) = sum(sum((output.DEMv_x(t_trim,xh)-...
%                               model.ideal_x(t_trim,xh)).^2));
% SSE.kalmanv_xh(i,j,k) = sum(sum((output.kalmfv_x(xh,t_trim)'-...
%                                  model.ideal_x(t_trim,xh)).^2));
% SSE.DEMv_xobs(i,j,k) = sum(sum((output.DEMv_x(t_trim,xobs)-...
%                                 model.ideal_x(t_trim,xobs)).^2));
% SSE.kalmanv_xobs(i,j,k) = sum(sum((output.kalmfv_x(xobs,t_trim)'-...
%                                    model.ideal_x(t_trim,xobs)).^2));
% % Weighted error of observable and hidden state (893 = 4.2/0.0047)
% % SSE.DEMv_xw(i,j,k) = SSE.DEMv_xobs(i,j,k)*893 + SSE.DEMv_xh(i,j,k);
% % fprintf('s: %9.4f\n',model.s);
% % fprintf('SSE: %9.4f\n\n',SSE.DEMv_x(i,j,k));
% fprintf('(%d,%d,%5.4f)\n',brain.p,brain.d,brain.s);
%         end
%     end
% end


%% Plot data
print_results(SSE,if_UIO,if_cause,xh);
if xh
    plot_results_xh(output,SSE,model,brain,if_UIO,if_cause,xh,t_trim);
else
    plot_results(output,model,brain,if_UIO,if_cause);
end
