%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEM observer main file
%
% Main script used to run DEM observer (D step of DEM) on experimental or
% synthetic data and compare it with a Kalman filter or Unknown Input
% Observer (UIO is not used in this project).
% 
% Results using this algorithm can be found in:
% 1. A. A. Meera and M. Wisse, "Free Energy Principle Based State and Input
%    Observer Design for Linear Systems with Colored Noise," 2020
%    American Control Conference (ACC), Denver, CO, USA, 2020,
%    pp. 5052-5058, doi: 10.23919/ACC45564.2020.9147581.
% 2. http://resolver.tudelft.nl/uuid:156157c6-d7f0-4dc1-a55a-b2e4ed66f1c2
% 
% Code source:     https://github.com/ajitham123/DEM_observer
% Original author: Ajith Anil Meera, TU Delft, CoR
% Adjusted by:     Dennis Benders, TU Delft, CoR
% Last modified:   20.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Important variables
% - Tt                Time vector (starting from 0)

% 'model' structure represents the generative process
% - model.t           Time vector starting from sampling time
% - model.sam_time    Sampling time
% - model.real_cause  Input (measured)          - dim: [nv x nt]
% - model.process_x   State (used as benchmark) - dim: [nt x nx]
% - model.process_y   Output (measured)         - dim: [nt x ny]
% - model.A           A matrix                  - dim: [nx x nx]
% - model.B           B matrix                  - dim: [nx x nv]
% - model.C           C matrix                  - dim: [ny x nx]
% - model.s           Noise smoothness
% - model.p           Embedding order of states and outputs
% - model.d           Embedding order of inputs
% - model.Pw          Precision matrix of process noise
% - model.Pz          Precision matrix of measurement noise
% - model.sigma_w     Standard deviation of process noise
% - model.sigma_z     Standard deviation of measurement noise
% - model.prior_cause Input prior

% 'brain' structure represeants the generative model
% - brain.nv          Number of inputs
% - brain.nx          Number of states
% - brain.ny          Number of outputs
% - brain.nt          Number of total time steps
% - brain.sigma_v     Standard deviation of inputs


%% Initialization
% MATLAB GUI
clear;
close all;
clc;

% Enable printing and plotting
if_print = 1;
if_plot  = 1;


%% Adjustable settings
% Choose whether inputs should also be estimated or not
% 0: inputs should be estimated
% 1: inputs are known (used in this project)
if_cause = 1;

% Set the index of the hidden state that is compared with an external
% reference. Setting xh = 0 means there is no hidden state
xh = 2;

% Embedding orders
model.p = 2;
model.d = 2;

% Input information
brain.sigma_v = 10;
prior_cause   = 0.01;

% Necessary intitialization without meaning when comparing DEM and Kalman,
% based on AR.Drone 2.0 flight data
if_dataset   = 1; %0 for synthetic data and 1 for AR.Drone 2.0 flight data

if_predict_y = 0;

if_UIO       = 0; %1 to use UIO (provided if_cause is set to 0. Might need
                  %additional software installation
UIO_gamma    = 125;
UIO_gains    = [1200 1200 1200 1400 1500];


% Get model and corresponding flight data
if if_dataset
    [Tt,model.sam_time,model.real_cause,model.process_x,model.process_y,...
     model.A,model.B,model.C,model.s,model.Pw,model.Pz] = ...
     get_ardrone2_flight_data;

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

    model.s = 0.5;
    model.sigma_w = exp(-4);
    model.sigma_z = exp(-4);
end


%% Start - obtain filter results for ranges of p, d and s values
% Uncomment this section and corresponding End section below to run the
% filters for different combinations of p, d and s values.
% TAKE CARE: uncommenting these sections only works for if_cause = 1 and
%            xh ~= 0.

% if_print = 0;
% if_plot = 0;
% 
% s_range = [1e-4:1e-4:1e-3,2e-3:1e-3:1.5e-2];
% p_range = 0:1:7;
% d_range = 0:1:7;
% 
% n_p = length(p_range);
% n_d = length(d_range);
% n_s = length(s_range);
% 
% SSE.DEMv_x       = zeros(n_p,n_d,n_s);
% SSE.kalmanv_x    = zeros(n_p,n_d,n_s);
% SSE.DEMv_xh      = zeros(n_p,n_d,n_s);
% SSE.kalmanv_xh   = zeros(n_p,n_d,n_s);
% SSE.DEMv_xobs    = zeros(n_p,n_d,n_s);
% SSE.kalmanv_xobs = zeros(n_p,n_d,n_s);
% 
% VAF.DEMv_x       = zeros(n_p,n_d,n_s);
% VAF.kalmanv_x    = zeros(n_p,n_d,n_s);
% VAF.DEMv_xh      = zeros(n_p,n_d,n_s);
% VAF.kalmanv_xh   = zeros(n_p,n_d,n_s);
% VAF.DEMv_xobs    = zeros(n_p,n_d,n_s);
% VAF.kalmanv_xobs = zeros(n_p,n_d,n_s);
% 
% for i = 1:n_p
%     for j = 1:n_d
%         for k = 1:n_s
% 
%             model.s = s_range(k);
%             model.p = p_range(i);
%             model.d = d_range(j);


%% Define various filter variables according to settings above
% Dimensions
brain.nv = size(model.B,2);
brain.ny = size(model.C,1);
brain.nx = size(model.A,1);
brain.nt = size(model.t,2);

% Smoothness
brain.s = model.s;

% Embedding orders
brain.p = model.p;
brain.d = model.d;

% Input prior
model.prior_cause = prior_cause*ones(brain.nv,brain.nt);


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
% 2: trim out estimates only at end to check filter convergence
if_t_trim = 2;
if if_t_trim == 1
    t_trim = brain.p+2:brain.nt-brain.p-2;
elseif if_t_trim == 2
    t_trim = 1:brain.nt-brain.p-2;
else
    t_trim = 1:brain.nt;
end

if if_cause
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
        xobs = 1:brain.nx;
        xobs(xh) = [];
        SSE.DEM.xobs    = sum(sum((output.DEM_x(t_trim,xobs)-...
                                    model.ideal_x(t_trim,xobs)).^2));
        SSE.kalman.xobs = sum(sum((output.kalman_x(xobs,t_trim)'-...
                                    model.ideal_x(t_trim,xobs)).^2));
    end
end


%% End - obtain filter results for ranges of p, d and s values
% SSE.DEMv_x(i,j,k)       = sum(sum((output.DEMv_x(t_trim,1:brain.nx)-...
%                                    model.ideal_x(t_trim,:)).^2));
% SSE.kalmanv_x(i,j,k)    = sum(sum((output.kalmfv_x(:,t_trim)'-...
%                                    model.ideal_x(t_trim,:)).^2));
% SSE.DEMv_xh(i,j,k)      = sum(sum((output.DEMv_x(t_trim,xh)-...
%                                    model.ideal_x(t_trim,xh)).^2));
% SSE.kalmanv_xh(i,j,k)   = sum(sum((output.kalmfv_x(xh,t_trim)'-...
%                                    model.ideal_x(t_trim,xh)).^2));
% SSE.DEMv_xobs(i,j,k)    = sum(sum((output.DEMv_x(t_trim,xobs)-...
%                                    model.ideal_x(t_trim,xobs)).^2));
% SSE.kalmanv_xobs(i,j,k) = sum(sum((output.kalmfv_x(xobs,t_trim)'-...
%                                    model.ideal_x(t_trim,xobs)).^2));
% 
% VAF.DEMv_x(i,j,k)       = get_vaf(output.DEMv_x(t_trim,1:brain.nx),...
%                                  model.ideal_x(t_trim,:));
% VAF.kalmanv_x(i,j,k)    = get_vaf(output.kalmfv_x(:,t_trim)',...
%                                  model.ideal_x(t_trim,:));
% VAF.DEMv_xh(i,j,k)      = get_vaf(output.DEMv_x(t_trim,xh),...
%                                  model.ideal_x(t_trim,xh));
% VAF.kalmanv_xh(i,j,k)   = get_vaf(output.kalmfv_x(xh,t_trim)',...
%                                  model.ideal_x(t_trim,xh));
% VAF.DEMv_xobs(i,j,k)    = get_vaf(output.DEMv_x(t_trim,xobs),...
%                                  model.ideal_x(t_trim,xobs));
% VAF.kalmanv_xobs(i,j,k) = get_vaf(output.kalmfv_x(xobs,t_trim)',...
%                                  model.ideal_x(t_trim,xobs));
% 
% fprintf('(%d,%d,%5.4f)\n',brain.p,brain.d,brain.s);
%         end
%     end
% end


%% Show results
% Print
if if_print
    print_results(SSE,if_cause,xh);
end

% Plot
if if_plot
    if xh
        plot_results_xh(output,SSE,model,brain,if_cause,xh,t_trim);
    else
        plot_results(output,model,brain,if_UIO,if_cause);
    end
end
