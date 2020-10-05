%% Initialisation
clc;
clear;
close all;


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
[xExp,xExpSimpleDer] = getFullState(expData);


%% Calculate MSE for white-box LTI system
[xWB,mseWB] = ltiStepSim(sysd,t,xExp,u,param);
% [xWB,mseWB] = ltiStepSim(sysd,t,xExpSimpleDer,u,param);


%% Calculate MSE for black-box LTI system
% Load and discretize system estimate
load sysBB_exp_24-7_7.mat;
ssModel = ssUnfilt;
syscBB = ss(ssModel.A,ssModel.B,ssModel.C,ssModel.D);
sysdBB = c2d(syscBB,param.sampleTime);

% Simulate LTI system
[xBB,mseBB] = ltiStepSim(sysdBB,t,xExp,u,param);


%% Plot results and compare with OptiTrack data
% quadrotor3DVisualization(t,x,'Simulated quadrotor movements');

figure('Name','Position and attitude');
subplot(3,2,1);
hold on;
plot(t,xWB(1,:));
plot(t,xBB(1,:));
plot(t,expData.output.otPos(1,:));
% legend('WB LTI model','OptiTrack');
legend('WB LTI model','BB LTI model','OptiTrack');
title('x');

subplot(3,2,3);
hold on;
plot(t,xWB(2,:));
plot(t,xBB(2,:));
plot(t,expData.output.otPos(2,:));
% legend('WB LTI model','OptiTrack');
legend('WB LTI model','BB LTI model','OptiTrack');
title('y');

subplot(3,2,5);
hold on;
plot(t,xWB(3,:));
plot(t,xBB(3,:));
plot(t,expData.output.otPos(3,:));
% legend('WB LTI model','OptiTrack');
legend('WB LTI model','BB LTI model','OptiTrack');
title('z');


subplot(3,2,2);
hold on;
plot(t,xWB(7,:));
plot(t,xBB(7,:));
plot(t,expData.output.otOrient(1,:));
% legend('WB LTI model','OptiTrack');
legend('WB LTI model','BB LTI model','OptiTrack');
title('\phi');

subplot(3,2,4);
hold on;
plot(t,xWB(8,:));
plot(t,xBB(8,:));
plot(t,expData.output.otOrient(2,:));
% legend('WB LTI model','OptiTrack');
legend('WB LTI model','BB LTI model','OptiTrack');
title('\theta');

subplot(3,2,6);
hold on;
plot(t,xWB(9,:));
plot(t,xBB(9,:));
plot(t,expData.output.otOrient(3,:));
% legend('WB LTI model','OptiTrack');
legend('WB LTI model','BB LTI model','OptiTrack');
title('\psi');



figure('Name','Linear and angular velocity');
subplot(3,2,1);
hold on;
plot(t,xWB(4,:));
plot(t,xBB(4,:));
plot(t,xExp(4,:));
% legend('WB LTI model','Finite differences approach');
legend('WB LTI model','BB LTI model','Finite differences approach');
title('v_x');

subplot(3,2,3);
hold on;
plot(t,xWB(5,:));
plot(t,xBB(5,:));
plot(t,xExp(5,:));
% legend('WB LTI model','Finite differences approach');
legend('WB LTI model','BB LTI model','Finite differences approach');
title('v_y');

subplot(3,2,5);
hold on;
plot(t,xWB(6,:));
plot(t,xBB(6,:));
plot(t,xExp(6,:));
% legend('WB LTI model','Finite differences approach');
legend('WB LTI model','BB LTI model','Finite differences approach');
title('v_z');


subplot(3,2,2);
hold on;
plot(t,xWB(10,:));
plot(t,xBB(10,:));
plot(t,xExp(10,:));
% legend('WB LTI model','Finite differences approach');
legend('WB LTI model','BB LTI model','Finite differences approach');
title('v_{\phi}');

subplot(3,2,4);
hold on;
plot(t,xWB(11,:));
plot(t,xBB(11,:));
plot(t,xExp(11,:));
% legend('WB LTI model','Finite differences approach');
legend('WB LTI model','BB LTI model','Finite differences approach');
title('v_{\theta}');

subplot(3,2,6);
hold on;
plot(t,xWB(12,:));
plot(t,xBB(12,:));
plot(t,xExp(12,:));
% legend('WB LTI model','Finite differences approach');
legend('WB LTI model','BB LTI model','Finite differences approach');
title('v_{\psi}');
