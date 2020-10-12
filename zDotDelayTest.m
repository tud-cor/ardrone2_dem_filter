%% Initialization
clear;
close all;
clc;


%% Select data
% Load pre-processed experiment data file
load hoverSpiralling25-100Hz15-120s.mat expData;

% System parameters
param.m           = 0.481;
param.ixx         = 3.4e-3;
param.ixy         = 0;
param.ixz         = 0;
param.iyy         = 4.0e-3;
param.iyz         = 0;
param.izz         = 6.9e-3;
param.irotor      = 2.030e-5;
param.g           = 9.81;
param.PwmToPwm    = 2.55;
param.PwmToOmegaR = [3.7,130.9];
param.cT          = [8.6e-6,-3.2e-4];
param.cTD         = [8.39e-6,-3.72e-5,-3.82e-2];
param.cTT         = [1.11e-5,-7.64e-4,4.12e-2];
param.cQ          = [2.4e-7,-9.9e-6];
param.ts          = expData.sampleTime;


% Calculate output data (vertical velocity zDot)
[xExp,xExpSimpleDer] = getFullState(expData);
zDot = xExpSimpleDer(6,:)';

% Calculate input data (thrust T)
rotorSpeed = expData.input.motor;
pwmToolbox = rotorSpeed/param.PwmToPwm;
omegaR     = param.PwmToOmegaR(1)*pwmToolbox+param.PwmToOmegaR(2);
n          = size(rotorSpeed,2);
T          = zeros(1,n);
tauPhi     = zeros(1,n);
tauTheta   = zeros(1,n);
tauPsi     = zeros(1,n);
for i = 1:dur
%     f           = param.cT(1)*omegaR(:,i).^2 + param.cT(2)*omegaR(:,i);
    f           = param.cTD(1)*omegaR(:,i).^2 + ...
                  param.cTD(2)*omegaR(:,i) + ...
                  param.cTD(3);
%     f           = param.cTT(1)*omegaR(:,i).^2 + ...
%                   param.cTT(2)*omegaR(:,i) + ...
%                   param.cTT(3);

    T(i)        = sum(f);
    tauPhi(i)   = sqrt(1/2)*param.l*(f(1)-f(2)-f(3)+f(4));
    tauTheta(i) = sqrt(1/2)*param.l*(-f(1)-f(2)+f(3)+f(4));
    tauPsi(i)   = param.cQ(1)*(sum(omegaR(1:2:3,i).^2)-...
                               sum(omegaR(2:2:4,i).^2)) + ...
                  param.cQ(2)*(sum(omegaR(1:2:3,i))-...
                               sum(omegaR(2:2:4,i)));
end
u = [T;tauPhi;tauTheta;tauPsi];



% Create iddata object
data = iddata(zDot',T',param.ts);

% Estimate delay input-to-output using iddata object
nk = delayest(data)
nk2 = delayest(data,1,1,0,50)



% Transform LIT model into ARMAX model
% sysP = idpoly(sysD);



% Simulate zDot dynamics for delay of 6
A    = 0;
B    = 1/param.m;
C    = 1;
D    = 0;
sysC = ss(A,B,C,D);
sysD = c2d(sysC,param.ts);

d  = 1;
nD = n-d+1;


tRef    = expData.input.time;
xRef    = zDot;
uRef    = T - param.m*param.g;

xSim    = zeros(1,n);
xSim(1) = xRef(1);
for i = 1:n-1
    xSim(:,i+1) = sysD.A*xRef(i) + sysD.B*uRef(i);
end


tRefD    = tRef(d:end);
xRefD    = xRef(d:end);
uRefD    = uRef(1:end-d+1);

xSimD    = zeros(1,nD);
xSimD(1) = xRefD(1);
for i = 1:nD-1
    xSimD(:,i+1) = sysD.A*xRefD(i) + sysD.B*uRefD(i);
end


% Plot simulation results
figure('Name','zDot simulation results');
plot(tRef,xRef);
hold on;
plot(tRef,xSim);
plot(tRefD,xSimD);
legend('zDot meas','zDot sim','zDot sim delay');