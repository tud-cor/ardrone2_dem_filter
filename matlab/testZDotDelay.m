%% Initialize MATLAB interface
clear;
close all;
clc;


%% Set parameters
floatTol = 1e-6;
plotXLim = [0,2];


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
param.l           = 0.178;
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
zDot = xExpSimpleDer(6,:);

% Calculate input data (thrust T)
rotorSpeed = expData.input.motor;
pwmToolbox = rotorSpeed/param.PwmToPwm;
omegaR     = param.PwmToOmegaR(1)*pwmToolbox+param.PwmToOmegaR(2);
n          = size(rotorSpeed,2);
T          = zeros(1,n);
tauPhi     = zeros(1,n);
tauTheta   = zeros(1,n);
tauPsi     = zeros(1,n);
for i = 1:n
    f           = param.cT(1)*omegaR(:,i).^2 + param.cT(2)*omegaR(:,i);
%     f           = param.cTD(1)*omegaR(:,i).^2 + ...
%                   param.cTD(2)*omegaR(:,i) + ...
%                   param.cTD(3);
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


%% Estimate delay using delayest
% Create iddata object
% nOffsetInput = linspace(1,200,200);
% for i = 1:length(nOffsetInput)
%     data = iddata(zDot(1:end-nOffsetInput(i))',T(nOffsetInput(i)+1:end)',param.ts);
% 
%     % Estimate delay input-to-output using iddata object
%     nk(i) = delayest(data);
%     nk2(i) = delayest(data,1,1,0,200);
% end
% plot(nOffsetInput,nk);
% hold on;
% plot(nOffsetInput,nk2);


%% Estimate delay using cross-correlation
maxLags = 200;
[c,lags] = xcorr(T,zDot*0.0832,maxLags);
plot(lags,c);


%% Align signals using alignsignals(__)
% [TA,zDotA,d] = alignsignals(T,zDot);
% [zDotA,TA,d] = alignsignals(zDot,T);


%% Simulate zDot dynamics for delay d
A    = 0;
B    = 1/param.m;
C    = 1;
D    = 0;
sysC = ss(A,B,C,D);
sysD = c2d(sysC,param.ts);

% Transform LIT model into ARMAX model
sysP = idpoly(sysD)

d  = 5;
nD = n-d;


tRef    = expData.input.time;
xRef    = zDot;
uRef    = T(1:end-1) - param.m*param.g;

xSim    = zeros(1,n);
xSim(1) = xRef(1);
for i = 1:n-1
    xSim(:,i+1) = sysD.A*xRef(i) + sysD.B*uRef(i);
end


tRefD    = tRef(d+1:end);
xRefD    = xRef(d+1:end);
uRefD    = uRef(1:end-d);

xSimD    = zeros(1,nD);
xSimD(1) = xRefD(1);
for i = 1:nD-1
    xSimD(:,i+1) = sysD.A*xRefD(i) + sysD.B*uRefD(i);
end


%% Calculate differences with respect to previous value
tRefDiff = tRef(1:end-1);
tRefDDiff = tRefD(1:end-1);
zDotDiff = setDiff(zDot,floatTol);
xSimDiff = setDiff(xSim,floatTol);
xSimDDiff = setDiff(xSimD,floatTol);


%% Plot simulation results
figure('Name','zDot simulation results');
plot(tRef,xRef);
hold on;
plot(tRef,xSim);
plot(tRefD,xSimD);
xlim(plotXLim);
legend('zDot meas','zDot sim','zDot sim delay');


%% Plot diff results
figure('Name','zDot simulation diff results');
subplot(2,1,1);
plot(tRefDiff,zDotDiff,'-o');
hold on;
plot(tRefDiff,xSimDiff,'-o');
plot(tRefDDiff,xSimDDiff,'-o');
legend('zDotDiff','xSimDiff','xSimDDiff');
xlim(plotXLim);
title('Changes in zDot');

subplot(2,1,2);
plot(tRefDiff,xSimDiff-zDotDiff,'-o');
hold on;
plot(tRefDDiff,xSimDDiff-zDotDiff(d+1:end),'-o');
yline(0);
xlim(plotXLim);
legend('xSimDiff','xSimDDiff');
title('Changes in zDot with respect to zDotDiff');


%% Function definitions
function y = setDiff(x,floatTol)
n = length(x);
y = zeros(1,n-1);
for i = 1:n-1
    y(i) = x(i+1)-x(i);
end
end

function y = setDiffD(x,floatTol)
n = length(x);
y = zeros(1,n-1);
for i = 1:n-1
    if abs(x(i+1)-x(i)) < floatTol
        y(i) = 0;
    elseif x(i+1)-x(i) < 0
        y(i) = -1;
    else
        y(i) = 1;
    end
end
end

function y = setDiffDU(x,floatTol)
n = length(x);
y = zeros(1,n);
for i = 1:n
    if abs(x(i)) < floatTol
        y(i) = 0;
    elseif x(i) < 0
        y(i) = -1;
    else
        y(i) = 1;
    end
end
end