%% Initialization
clear;
close all;
clc;


%% System parameters
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
% cT(1)*omegaR^2 + cT(2)*omegaR
param.cT            = [8.6e-6,-3.2e-4];
% cQ(1)*omegaR^2 + cQ(2)*omegaR
param.cQ            = [2.4e-7,-9.9e-6];


%% LTI state-space description and discretize
% Construct continuous-time linearised state space system for z
nu = 1;
nx = 2;
ny = 1;

A = [0,1;0,0];
B = [0;1/param.m];
C = [1,0];
D = 0;

% Linearized system analysis
lambda = eig(A);
con = ctrb(A,B);
nUncon = size(con,1) - rank(con);
obs = obsv(A,C);
nUnobs = size(obs,2) - rank(obs);

sysC = ss(A,B,C,D);


%% Load experiment data
load expData7_24_3.mat;

t    = expData.input.time;
ts   = expData.sampleTime;
nDur = length(t);


%% Construct input data
rotorSpeed  = expData.input.navMotor;
pwmToolbox  = rotorSpeed/param.PwmToPwm;
omegaR      = param.PwmToOmegaR(1)*pwmToolbox+param.PwmToOmegaR(2);
f           = zeros(4,1);
T           = zeros(1,nDur);
for i = 1:nDur
    f	 = param.cT(1)*omegaR(:,i).^2 + param.cT(2)*omegaR(:,i);
    T(i) = sum(f);
end
u = T;

% Convert to input around operating point
uOp = param.m*param.g;
uLin = u - uOp;


%% Construct ground truth state data
% Remove sensor offsets
tAvg = 3;
[~,otAvgEnd] = min(abs(expData.origData.otTime-tAvg));
x = expData.output.otPos(3,:) - mean(expData.origData.otPos(3,1:otAvgEnd));

% Convert to state around operating point
xOp = mean(x);
xLin = x - xOp;


%% Construct output data
% Remove sensor offsets
[~,odomAvgEnd] = min(abs(expData.origData.odomTime-tAvg));
y = expData.output.odomPos(3,:) - ...
    mean(expData.output.odomPos(3,1:odomAvgEnd));

% Convert to output around operating point
yOp = mean(x);
yLin = y - yOp;

% Ensure that output and ground truth data are aligned at the start
yLin = yLin + (xLin(1)-yLin(1));

% Calculate noise properties of output noise
ySigma = 0.0406;
yS = 0.1;
yS2 = 0.0119;
% ySigma = zeros(ny,ny);
% yS     = zeros(ny,1);
% [ySigma,yS] = estimateNoiseCharacteristics(t,yLin,1,1);
% yS2 = estimateSmoothness(t,yLin);


%% Save data
filename = sprintf('demCode/ardrone2FlightDataZ_%s',...
                   datestr(now,'dd-mm-yyyy_HH-MM'));
save(filename,...
     't','ts',...
     'uLin','xLin','yLin','ySigma','yS','yS2',...
     'A','B','C');
