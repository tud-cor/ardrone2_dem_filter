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



%% LTI state-space description and discretize - complete
% Construct continuous-time linearised state space system
% Complete system
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

sysC = ss(A,B,C,D);


%% LTI state-space description and discretize - option 3
% Construct continuous-time linearised state space system
xSel = [1,2,4,5,7,8,10,11];
uSel = [2,3];

nx = length(xSel);
nu = length(uSel);
ny = 4;

A3 = A(xSel,xSel);
B3 = B(xSel,uSel);
C3 = [0,0,1,0,0,0,0,0;...
     0,0,0,1,0,0,0,0;...
     0,0,0,0,0,0,1,0;...
     0,0,0,0,0,0,0,1];
D3 = zeros(ny,nu);

nu = 2;
nx = 8;
ny = 4;


% Linearized system analysis
lambda3 = eig(A3);
con3 = ctrb(A3,B3);
nUncon3 = size(con3,1) - rank(con3);
obs3 = obsv(A3,C3);
nUnobs3 = size(obs3,2) - rank(obs3);

sysC3 = ss(A3,B3,C3,D3);


%% LTI state-space description and discretize - option 4
% Construct continuous-time linearised state space system
% (option 4 in notes: only xDot,yDot,phi,theta,phiDot,thetaDot)
xSel = [4,5,7,8,10,11];
uSel = [2,3];

nx = length(xSel);
nu = length(uSel);
ny = 4;

A4 = A(xSel,xSel);
B4 = B(xSel,uSel);
C4 = [1,0,0,0,0,0;
      0,1,0,0,0,0;
      0,0,0,0,1,0;
      0,0,0,0,0,1];
D4 = zeros(ny,nu);


% Linearized system analysis
lambda4 = eig(A4);
con4 = ctrb(A4,B4);
nUncon4 = size(con4,1) - rank(con4);
obs4 = obsv(A4,C4);
nUnobs4 = size(obs4,2) - rank(obs4);

sysC4 = ss(A4,B4,C4,D4);


%% LTI state-space description and discretize - option 5
% Construct continuous-time linearised state space system
% (option 5 in notes: only xDot,theta,thetaDot)
xSel = [4,8,11];
uSel = 3;

nx = length(xSel);
nu = length(uSel);
ny = 2;

A5 = A(xSel,xSel);
B5 = B(xSel,uSel);
C5 = [1,0,0;
      0,0,1];
D5 = zeros(ny,nu);


% Linearized system analysis
lambda5 = eig(A5);
con5 = ctrb(A5,B5);
nUncon5 = size(con5,1) - rank(con5);
obs5 = obsv(A5,C5);
nUnobs5 = size(obs5,2) - rank(obs5);

sysC5 = ss(A5,B5,C5,D5);


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
yS2 = 0.0065;
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
