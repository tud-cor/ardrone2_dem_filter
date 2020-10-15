%% TODO
% Implement MSE and VAF in separate error function

%% Initialisation
clc;
clear;
close all;


%% Parameters to adjust
% Derivative selection (how are the derivatives constructed?)
% 0: simple derivatives (using current point and next point TODO)
% 1: derivatives constructed using finite differences approach
derSel = 1;

% Model selection (which model has to be simulated?)
% [white-box ; grey-box ; black-box ; nonlinear]
modSel = [1;0;0;0];


%% Check model selection
nModSel = sum(modSel);
if nModSel == 0
    error(['Please select at least one model to compare with ' ...
            'OptiTrack data.']);
end


%% Select simulation data
% Load pre-processed simulation data file
load hoverSpiralling25-100Hz15-120s.mat expData;

% Select samples to use in simulation
startSample = 1;
endSample = 2600;

expData.output.otTime = expData.output.otTime(startSample:endSample);
expData.output.otPos = expData.output.otPos(:,startSample:endSample);
expData.output.otOrient = expData.output.otOrient(:,startSample:endSample);

nOffset = expData.sampleTime/expData.sampleTimeHighFreq;
[~,sIdx] = min(abs(expData.output.highFreq.otTime-...
                   expData.output.otTime(1)));
[~,eIdx] = min(abs(expData.output.highFreq.otTime-...
                   expData.output.otTime(end)));
expData.output.highFreq.otTime = ...
    expData.output.highFreq.otTime(sIdx-nOffset:eIdx+nOffset);
expData.output.highFreq.otPos = ...
    expData.output.highFreq.otPos(:,sIdx-nOffset:eIdx+nOffset);
expData.output.highFreq.otOrient = ...
    expData.output.highFreq.otOrient(:,sIdx-nOffset:eIdx+nOffset);

expData.input.time = expData.input.time(startSample:endSample);
expData.input.motor = expData.input.motor(:,startSample:endSample);


%% System parameters
% Floating-point accuracy
param.timeThres     = 1e-10;

% Position at the ground accuracy
param.groundThres   = 1e-3;     %m

% Recorded data accuracy
param.sampleTime    = expData.sampleTime;   %s

% Environmental constants
param.g             = 9.81;     %m/s^2
param.densityAir    = 1.2;      %kg/m^3 (for room temperature
                                %        ~20 degree Celcius)

% Mass and inertia
%                   |ixx        ixy         ixz         |
% Inertia matrix:   |iyx = ixy  iyy         iyz         |
%                   |izx = ixz  izy = iyz   izz         |
% Rotor inertia only has izz component
param.m             = 0.481;	%kg
param.ixx           = 3.4e-3;	%kgm^2
param.ixy           = 0;        %kgm^2
param.ixz           = 0;        %kgm^2
param.iyy           = 4.0e-3;   %kgm^2
param.iyz           = 0;        %kgm^2
param.izz           = 6.9e-3;   %kgm^2
param.irotor        = 2.030e-5;	%kgm^2 TODO: value from Q. Li (2014)

% Dimensions
param.l             = 0.178;	%m

% Thrust and torque coefficients
%PWM-PWM relation: PWM_ardrone/navdata = 2.55*PWM_toolbox
param.PwmToPwm      = 2.55;
% omegaR = PwmToOmegaR(1)*pwm + PwmToOmegaR(2)
param.PwmToOmegaR   = [3.7,130.9];
% cT(1)*omegaR^2 + cT*omegaR
param.cT            = [8.6e-6,-3.2e-4];
% cQ(1)*omegaR^2 + cq*omegaR
param.cQ            = [2.4e-7,-9.9e-6];


%% LTI state-space description and discretize
% Construct continuous-time linearised state space system
nu = 4;
nx = 12;
ny = 6;
A = [0, 0, 0, 1, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 1, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , param.g, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, -param.g, 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 1, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0];
B = [0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     1/param.m, 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 1/param.ixx, 0          , 0;
     0        , 0          , 1/param.iyy, 0;
     0        , 0          , 0          , 1/param.izz];
C = zeros(ny,nx);
C(1:3,1:3) = eye(3);
C(4:6,7:9) = eye(3);
D = zeros(ny,nu);

% Linearized system analysis
lambda = eig(A);
con = ctrb(A,B);
nUncon = size(con,1) - rank(con);
obs = obsv(A,C);
nUnobs = size(obs,2) - rank(obs);

sysc = ss(A,B,C,D);

% Construct discrete-time linearised state space system
sysd = c2d(sysc,param.sampleTime);


%% Simulation parameters
t           = expData.input.time;

% Construct rotor speed input
rotorSpeed  = expData.input.motor;

% Construct input from rotor speeds
pwmToolbox  = rotorSpeed/param.PwmToPwm;
omegaR      = param.PwmToOmegaR(1)*pwmToolbox+param.PwmToOmegaR(2);
dur         = length(expData.input.time);
f           = zeros(4,1);
T           = zeros(1,dur);
tauPhi      = zeros(1,dur);
tauTheta    = zeros(1,dur);
tauPsi      = zeros(1,dur);
for i = 1:dur
    f           = param.cT(1)*omegaR(:,i).^2 + param.cT(2)*omegaR(:,i);

    T(i)        = sum(f);
    tauPhi(i)   = sqrt(1/2)*param.l*(f(1)-f(2)-f(3)+f(4));
    tauTheta(i) = sqrt(1/2)*param.l*(-f(1)-f(2)+f(3)+f(4));
    tauPsi(i)   = param.cQ(1)*(sum(omegaR(1:2:3,i).^2)-...
                               sum(omegaR(2:2:4,i).^2)) + ...
                  param.cQ(2)*(sum(omegaR(1:2:3,i))-...
                               sum(omegaR(2:2:4,i)));
end
u = [T;tauPhi;tauTheta;tauPsi];


%% Construct full state from OptiTrack data
% TAKE CARE The system output (OptiTrack) data is used to compare with the
% simulated states of the model, since it is assumed that the OptiTrack
% data does not contain a lot of noise
[xExpFinDiff,xExpSimpleDer] = getFullState(expData);
if ~derSel
    xExp = xExpFinDiff;
else
    xExp = xExpSimpleDer;
end



%% Calculate MSE for white-box LTI system
if modSel(1)
    [xWB,mseWB] = ltiStepSim(sysd,t,xExp,u,param);
end


%% Calculate MSE for grey-box LTI system
if modSel(2)
    % Load and discretize system estimate
    load sysGB_exp_24-7_7.mat;
    sysdGB = c2d(syscGB,param.sampleTime);

    % Simulate LTI system
    [xGB,mseGB] = ltiStepSim(sysdGB,t,xExp,u,param);
end


%% Calculate MSE for black-box LTI system
if modSel(3)
    % Load and discretize system estimate
    load sysBB_exp_24-7_7.mat;
    ssModel = ssUnfilt;
    syscBB = ss(ssModel.A,ssModel.B,ssModel.C,ssModel.D);
    sysdBB = c2d(syscBB,param.sampleTime);

    % Simulate LTI system
    [xBB,mseBB] = ltiStepSim(sysdBB,t,xExp,u,param);
end


%% Calculate result for nonlinear system (using ODE45)
if modSel(4)
    xNonlin = zeros(nx,dur);
    xNonlin(:,1) = xExpSimpleDer(:,1);
    for i = 1:dur-1
        tic;
        x0 = xExpSimpleDer(:,i);
        tspan       = [t(i),t(i+1)];
        [tNonlinSim,xNonlinSim] = ode15s(@(tNonlinSim,xNonlinSim) ...
                                  nonlinSim(tNonlinSim,t,xNonlinSim,u,...
                                            omegaR,param),tspan,x0);
        xNonlin(:,i+1) = xNonlinSim(end,:);
        toc
    end
end


%% Plot results and compare with OptiTrack data
% quadrotor3DVisualization(t,x,'Simulated quadrotor movements');

% Construct matrix with state values in the following format
% (for selected models)
% [| WB state 1 | GB state 1 | ... | WB state 2 | GB state 2 | ...]
x = zeros(dur,nModSel*nx);
legend1 = cell(nModSel+1,1);
legend1{1} = 'OptiTrack';
legend2 = cell(nModSel+1,1);
if ~derSel
    legend2{1} = 'Simple derivative';
else
    legend2{1} = 'Derivative using finite differences';
end
nModPlotted = 0;
if modSel(1)
    for i = 1:nx
        x(:,(i-1)*nModSel+nModPlotted+1) = xWB(i,:)';
    end
    legend1{nModPlotted+2} = 'WB LTI model';
    legend2{nModPlotted+2} = 'WB LTI model';
    nModPlotted = nModPlotted + 1;
end
if modSel(2)
    for i = 1:nx
        x(:,(i-1)*nModSel+nModPlotted+1) = xGB(i,:)';
    end
    legend1{nModPlotted+2} = 'GB LTI model';
    legend2{nModPlotted+2} = 'GB LTI model';
    nModPlotted = nModPlotted + 1;
end
if modSel(3)
    for i = 1:nx
        x(:,(i-1)*nModSel+nModPlotted+1) = xBB(i,:)';
    end
    legend1{nModPlotted+2} = 'BB LTI model';
    legend2{nModPlotted+2} = 'BB LTI model';
    nModPlotted = nModPlotted + 1;
end
if modSel(4)
    for i = 1:nx
        x(:,(i-1)*nModSel+nModPlotted+1) = xNonlin(i,:)';
    end
    legend1{nModPlotted+2} = 'Nonlinear model';
    legend2{nModPlotted+2} = 'Nonlinear model';
    nModPlotted = nModPlotted + 1;
end


figure('Name','Position and attitude');
subplot(3,2,1);
plot(t,expData.output.otPos(1,:));
hold on;
plot(t,x(:,1:nModSel));
legend(legend1);
title('x');
% plot(t,xWB(1,:));
% plot(t,xGB(1,:));
% plot(t,xBB(1,:));
% plot(t,xNonlin(1,:));

subplot(3,2,3);
plot(t,expData.output.otPos(2,:));
hold on;
plot(t,x(:,nModSel+1:nModSel+nModSel));
legend(legend1);
title('y');
% plot(t,xWB(2,:));
% plot(t,xGB(2,:));
% plot(t,xBB(2,:));
% plot(t,xNonlin(2,:));

subplot(3,2,5);
plot(t,expData.output.otPos(3,:));
hold on;
plot(t,x(:,2*nModSel+1:2*nModSel+nModSel));
legend(legend1);
title('z');
% plot(t,xWB(3,:));
% plot(t,xGB(3,:));
% plot(t,xBB(3,:));
% plot(t,xNonlin(3,:));

subplot(3,2,2);
plot(t,expData.output.otOrient(1,:));
hold on;
plot(t,x(:,6*nModSel+1:6*nModSel+nModSel));
legend(legend1);
title('\phi');
% plot(t,xWB(7,:));
% plot(t,xGB(7,:));
% plot(t,xBB(7,:));
% plot(t,xNonlin(7,:));

subplot(3,2,4);
plot(t,expData.output.otOrient(2,:));
hold on;
plot(t,x(:,7*nModSel+1:7*nModSel+nModSel));
legend(legend1);
title('\theta');
% plot(t,xWB(8,:));
% plot(t,xGB(8,:));
% plot(t,xBB(8,:));
% plot(t,xNonlin(8,:));

subplot(3,2,6);
plot(t,expData.output.otOrient(3,:));
hold on;
plot(t,x(:,8*nModSel+1:8*nModSel+nModSel));
legend(legend1);
title('\psi');
% plot(t,xWB(9,:));
% plot(t,xGB(9,:));
% plot(t,xBB(9,:));
% plot(t,xNonlin(9,:));


figure('Name','Linear and angular velocity');
subplot(3,2,1);
plot(t,xExp(4,:));
hold on;
plot(t,x(:,3*nModSel+1:3*nModSel+nModSel));
legend(legend2);
title('v_x');
% plot(t,xWB(4,:));
% plot(t,xGB(4,:));
% plot(t,xBB(4,:));
% plot(t,xNonlin(4,:));

subplot(3,2,3);
plot(t,xExp(5,:));
hold on;
plot(t,x(:,4*nModSel+1:4*nModSel+nModSel));
legend(legend2);
title('v_y');
% plot(t,xWB(5,:));
% plot(t,xGB(5,:));
% plot(t,xBB(5,:));
% plot(t,xNonlin(5,:));

subplot(3,2,5);
plot(t,xExp(6,:));
hold on;
plot(t,x(:,5*nModSel+1:5*nModSel+nModSel));
legend(legend2);
title('v_z');
% plot(t,xWB(6,:));
% plot(t,xGB(6,:));
% plot(t,xBB(6,:));
% plot(t,xNonlin(6,:));

subplot(3,2,2);
plot(t,xExp(10,:));
hold on;
plot(t,x(:,9*nModSel+1:9*nModSel+nModSel));
legend(legend2);
title('v_{\phi}');
% plot(t,xWB(10,:));
% plot(t,xGB(10,:));
% plot(t,xBB(10,:));
% plot(t,xNonlin(10,:));

subplot(3,2,4);
plot(t,xExp(11,:));
hold on;
plot(t,x(:,10*nModSel+1:10*nModSel+nModSel));
legend(legend2);
title('v_{\theta}');
% plot(t,xWB(11,:));
% plot(t,xGB(11,:));
% plot(t,xBB(11,:));
% plot(t,xNonlin(11,:));

subplot(3,2,6);
plot(t,xExp(12,:));
hold on;
plot(t,x(:,11*nModSel+1:11*nModSel+nModSel));
legend(legend2);
title('v_{\psi}');
% plot(t,xWB(12,:));
% plot(t,xGB(12,:));
% plot(t,xBB(12,:));
% plot(t,xNonlin(12,:));
