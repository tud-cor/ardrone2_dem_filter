%% Initialisation
clc;
clear;
close all;


%% Select simulation data
% Load pre-processed simulation data file
load hoverSpiralling25-100Hz.mat expData;

% Select samples to use in simulation
startSample = 1;
endSample = 250;

expData.state.otTime = expData.state.otTime(startSample:endSample);
expData.state.otPos = expData.state.otPos(:,startSample:endSample);
expData.state.otOrient = expData.state.otOrient(:,startSample:endSample);

nOffset = expData.sampleTime/expData.sampleTimeHighFreq;
[~,sIdx] = min(abs(expData.state.highFreq.otTime-expData.state.otTime(1)));
[~,eIdx] = min(abs(expData.state.highFreq.otTime-...
                   expData.state.otTime(end)));
expData.state.highFreq.otTime = ...
    expData.state.highFreq.otTime(sIdx-nOffset:eIdx+nOffset);
expData.state.highFreq.otPos = ...
    expData.state.highFreq.otPos(:,sIdx-nOffset:eIdx+nOffset);
expData.state.highFreq.otOrient = ...
    expData.state.highFreq.otOrient(:,sIdx-nOffset:eIdx+nOffset);

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
n = 12;
l = 4;
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
C = zeros(n,n);
C(1:3,1:3) = eye(3);
% C(4:6,4:6) = eye(3);
C(7:9,7:9) = eye(3);
% C(10:12,10:12) = eye(3);
D = zeros(n,l);

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
[xExp,xExpSimpleDer] = getFullState(expData);


%% Calculate MSE for LTI system
[x,mse] = ltiStepSim(sysd,t,xExp,u,param);
% [x,mse] = ltiStepSim(sysd,t,xExpSimpleDer,u,param);


%% Plot results and compare with OptiTrack data
quadrotor3DVisualization(t,x,'Simulated quadrotor movements');

figure('Name','Position and attitude');
subplot(3,2,1);
hold on;
plot(t,x(1,:));
plot(t,expData.state.otPos(1,:));
legend('LTI','OptiTrack');
title('x');

subplot(3,2,3);
hold on;
plot(t,x(2,:));
plot(t,expData.state.otPos(2,:));
legend('LTI','OptiTrack');
title('y');

subplot(3,2,5);
hold on;
plot(t,x(3,:));
plot(t,expData.state.otPos(3,:));
legend('LTI','OptiTrack');
title('z');


subplot(3,2,2);
hold on;
plot(t,x(7,:));
plot(t,expData.state.otOrient(1,:));
legend('LTI','OptiTrack');
title('\phi');

subplot(3,2,4);
hold on;
plot(t,x(8,:));
plot(t,expData.state.otOrient(2,:));
legend('LTI','OptiTrack');
title('\theta');

subplot(3,2,6);
hold on;
plot(t,x(9,:));
plot(t,expData.state.otOrient(3,:));
legend('LTI','OptiTrack');
title('\psi');



figure('Name','Linear and angular velocity');
subplot(3,2,1);
hold on;
plot(t,x(4,:));
plot(t,xExp(4,:));
legend('LTI','Finite differences approach');
title('v_x');

subplot(3,2,3);
hold on;
plot(t,x(5,:));
plot(t,xExp(5,:));
legend('LTI','Finite differences approach');
title('v_y');

subplot(3,2,5);
hold on;
plot(t,x(6,:));
plot(t,xExp(6,:));
legend('LTI','Finite differences approach');
title('v_z');


subplot(3,2,2);
hold on;
plot(t,x(10,:));
plot(t,xExp(10,:));
legend('LTI','Finite differences approach');
title('v_{\phi}');

subplot(3,2,4);
hold on;
plot(t,x(11,:));
plot(t,xExp(11,:));
legend('LTI','Finite differences approach');
title('v_{\theta}');

subplot(3,2,6);
hold on;
plot(t,x(12,:));
plot(t,xExp(12,:));
legend('LTI','Finite differences approach');
title('v_{\psi}');
