%% Initialization
clear;
close all;
clc;

% Parameters
degToRad = 2*pi/180;


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
A = [0, 0, 0, 1, 0, 0, 0,          0,       0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0,          0,       0, 0, 0, 0;
     0, 0, 0, 0, 0, 1, 0,          0,       0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0,          param.g, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, -param.g,   0,       0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0,          0,       0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0,          0,       0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 0,          0,       0, 0, 1, 0;
     0, 0, 0, 0, 0, 0, 0,          0,       0, 0, 0, 1;
     0, 0, 0, 0, 0, 0, 0,          0,       0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0,          0,       0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0,          0,       0, 0, 0, 0];
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


%% Load experiment data
load expData7_24_8_mod5.mat;

t    = expData.input.time;
ts   = expData.sampleTime;
nDur = length(t);


%% Construct input data
rotorSpeed  = expData.input.navMotor;
pwmToolbox  = rotorSpeed/param.PwmToPwm;
omegaR      = param.PwmToOmegaR(1)*pwmToolbox+param.PwmToOmegaR(2);
f           = zeros(4,1);
tauTheta    = zeros(1,nDur);
for i = 1:nDur
    f	 = param.cT(1)*omegaR(:,i).^2 + param.cT(2)*omegaR(:,i);
    tauTheta(i) = sqrt(1/2)*param.l*(-f(1)-f(2)+f(3)+f(4));
end
u = tauTheta;

% Convert to input around operating point
uOp = 0;
uLin = u - uOp;

% TODO Assumed that input can be shifted, such that the amount of positive
% u samples is equal to the amount of positive state derivatives
uLin = uLin - 0.022;
% uLin = uLin - 5;
% 
% cntU = 0;
% for i = 1:length(uLin)-1
% if uLin(i) > 0
% cntU = cntU + 1;
% end
% end
% 
% q = expData.output.imuVAng(2,:);
% p = 1;
% o = 6;
% nOffset = ceil((p+0)/2);
% E = f_finitediffmat(ts,1,6,1,'c');
% for i = nOffset+1:length(q)-nOffset
%     a(i) = E(2,:)*q(i-nOffset:i+nOffset)';
% end
% cntA = 0;
% for i = 1:length(a)
% if a(i) > 0
% cntA = cntA + 1;
% end
% end


%% Construct ground truth state data
% Scale orientation and remove sensor offsets
tAvg = 3;
[~,otAvgEnd] = min(abs(expData.origData.otTime-tAvg));

psiOffset = mean(expData.origData.otOrient(3,1:otAvgEnd));
otOrientC = rotz(-psiOffset/pi*180)*expData.output.otOrient + [0;0;pi/2];

x = otOrientC(2,:) - mean(expData.origData.otOrient(2,1:otAvgEnd));

% [~,navAvgEnd] = min(abs(expData.origData.navTime - expData.origData.navTime - tAvg));
% x = expData.output.navRot(2,:)*degToRad - ...
%     mean(expData.origData.navRot(2,1:navAvgEnd))*degToRad;

% Convert to state around operating point
xOp = [0;0;0];
% xLin = [zeros(1,nDur);x;zeros(1,nDur)] - xOp;
xLin = [expData.output.navVLin(1,:)/1000;x;expData.output.imuVAng(2,:)] - xOp;


%% Construct output data
% Remove sensor offsets
% TODO Assume offset 0 for xDot and q (approximately equal to thetaDot)!
y = [expData.output.navVLin(1,:)/1000;expData.output.imuVAng(2,:)];

% Convert to output around operating point
yOp = [0;0];
yLin = y - yOp;

% Ensure that output and ground truth data are aligned at the start
% yLin = yLin + (xLin(1)-yLin(1));

% Calculate noise properties of output noise
% TODO Assume:
% - Gaussian filter validity
% - Same sigma and s for each output and state
ySigma = 6.9e-4;
yS = 0.0065;


%% Save data
filename = sprintf('demCode/ardrone2FlightData5_%s',...
                   datestr(now,'dd-mm-yyyy_HH-MM'));

A = A5;
B = B5;
C = C5;

save(filename,...
     't','ts',...
     'uLin','xLin','yLin','ySigma','yS',...
     'A','B','C');
