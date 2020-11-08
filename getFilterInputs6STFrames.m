%% Initialization
clear;
close all;
clc;

% Parameters
degToRad = 2*pi/180; %conversion from degrees to radians
pwmEq    = 169.5916; %Parrot battery PWM equilibrium
% pwmEq    = 171.4937; %Akku-King battery PWM equilibrium

% Changeable
% Model input definition
% 0: PWM values as input
% 1: using PWM values, derive torque around x-axis and use that as input
uSelect = 0;

% Selection of PWM-thrust coefficients, see below

% State and input selection, see below


%% System parameters
% Environmental constants
g      = 9.81;     %m/s^2

% Mass and inertia
%                   |ixx        ixy         ixz         |
% Inertia matrix:   |iyx = ixy  iyy         iyz         |
%                   |izx = ixz  izy = iyz   izz         |
% Rotor inertia only has izz component
m      = 0.481;	%kg
ixx    = 3.4e-3;	%kgm^2
ixy    = 0;        %kgm^2
ixz    = 0;        %kgm^2
iyy    = 4.0e-3;   %kgm^2
iyz    = 0;        %kgm^2
izz    = 6.9e-3;   %kgm^2

% Dimensions
l      = 0.178;	%m

% Thrust and torque coefficients
% omegaR = cM(1)*pwm_toolbox + cM(2)
cM = [3.7,130.9];

% omegaR = cA(1)*pwm_ardrone + cA(2)
cA = [cM(1)/2.55,cM(2)];

% cT(1)*omegaR^2 + cT(2)*omegaR
cTO = [8.6e-6,-3.2e-4]; %Own work
cTEs = [1.28e-05,-1.68e-3]; %Estimated

% cQ(1)*omegaR^2 + cQ(2)*omegaR
cQ = [2.4e-7,-9.9e-6];


% Choose coefficients
cT = cTEs;

% Determine derivative terms for thrust and torque w.r.t. PWM
cTDer      = 2*cT(1)*cA(1)^2*pwmEq + 2*cT(1)*cA(1)*cA(2) + cT(2)*cA(1);
cTPhiDer   = sqrt(2)/2*l*(2*cT(1)*cA(1)^2*pwmEq + ...
                          2*cT(1)*cA(1)*cA(2)+cT(2)*cA(1));
cTThetaDer = sqrt(2)/2*l*(2*cT(1)*cA(1)^2*pwmEq + ...
                          2*cT(1)*cA(1)*cA(2)+cT(2)*cA(1));
cTPsiDer   = 2*cQ(1)*cA(1)^2*pwmEq + 2*cQ(1)*cA(1)*cA(2) + cQ(2)*cA(1);

% Construct elements of B-matrix
bZDot     = 1/m*cTDer;
bPhiDot   = 1/ixx*cTPhiDer;
bThetaDot = 1/iyy*cTThetaDer;
bPsiDot   = 1/izz*cTPsiDer;


%% Load experiment data
load expData10_29_25_mod6.mat;

t    = expData.output.time;
ts   = expData.sampleTime;
nDur = length(t);


%% LTI state-space description and discretize - complete
% Construct continuous-time linearised state space system
% Complete system
nu = 4;
nx = 12;
ny = 6;
A = [0, 0, 0, 1, 0, 0, 0,  0, 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0,  0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 1, 0,  0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0,  g, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, -g, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0,  0, 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0];
if ~uSelect
    B = [0,          0,          0,         0;
         0,          0,          0,         0;
         0,          0,          0,         0;
         0,          0,          0,         0;
         0,          0,          0,         0;
         bZDot,      bZDot,      bZDot,     bZDot;
         0,          0,          0,         0;
         0,          0,          0,         0;
         0,          0,          0,         0;
         bPhiDot,    -bPhiDot,   -bPhiDot,  bPhiDot;
         -bThetaDot, -bThetaDot, bThetaDot, bThetaDot;
         bPsiDot,    -bPsiDot,   bPsiDot,   -bPsiDot];
else
    B = [0, 0,     0,     0;
         0, 0,     0,     0;
         0, 0,     0,     0;
         0, 0,     0,     0;
         0, 0,     0,     0;
         m, 0,     0,     0;
         0, 0,     0,     0;
         0, 0,     0,     0;
         0, 0,     0,     0;
         0, 1/ixx, 0,     0;
         0, 0,     1/iyy, 0;
         0, 0,     0,     1/izz];
end
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


%% LTI state-space description and discretize - option 6
% Construct continuous-time linearised state space system
% (option 6 in notes: only y,yDot,phi,phiDot)
xSel = [7,10];
if ~uSelect
    uSel = 1:4;
else
    uSel = 2;
end

nx = length(xSel);
nu = length(uSel);
ny = nx;

A6 = A(xSel,xSel);
B6 = B(xSel,uSel);
C6 = eye(nx);
D6 = zeros(ny,nu);

% Linearized system analysis
lambda6 = eig(A6);
con6    = ctrb(A6,B6);
nUncon6 = size(con6,1) - rank(con6);
obs6    = obsv(A6,C6);
nUnobs6 = size(obs6,2) - rank(obs6);


%% Construct input data
pwm = expData.input.navMotor;
if ~uSelect
    u = pwm;
    uOp = pwmEq;
else
    u = zeros(3,nDur); %ignoring yaw
    for i = 1:nDur
        f	 = cT(1)*cA(1)^2*pwm(:,i).^2 + ...
               (2*cT(1)*cA(1)*cA(2) + cT(2)*cA(1))*pwm(:,i) + ...
               kron(ones(4,1),cT(1)*cA(2)^2 + cT(2)*cA(2));
        u(1,i) = f(1) + f(2) + f(3) + f(4);
        u(2,i) = sqrt(1/2)*l*(f(1)-f(2)-f(3)+f(4));
        u(3,i) = sqrt(1/2)*l*(-f(1)-f(2)+f(3)+f(4));
    end
    u = u(uSel,:);
    uOp = [m*g;0;0];
    uOp = uOp(uSel);
end

% Convert to input around operating point
uLin = u - uOp;


%% Construct ground truth output data
yPos   = expData.output.otPos(2,:);
yDot   = expData.output.navVLin(2,:);
phi    = expData.output.otOrient(3,:);
phiDot = expData.output.imuVAng(1,:);
% y = [yPos;yDot;phi;phiDot];
% y = [yPos;yDot];
y = [phi;phiDot];

% Convert to output around operating point
% yOp = [mean(yPos);0;0;0];
% yOp = [mean(yPos);0];
yOp = [0;0];
yLin = y - yOp;

% Set state data equal to output data
xLin = yLin;
% xLin = [yDot;phi;phiDot];


%% Estimate process noise properties
% Discretize system
sysC = ss(A6,B6,C6,D6);
sysD = c2d(sysC,ts);

% Calculate process noise
w = zeros(nx,length(t)-1);
for i = 1:length(t)-1
    w(:,i) = xLin(:,i+1) - sysD.A*xLin(:,i) - sysD.B*uLin(:,i);
end

% Calculate precision matrix of process noise
% wCov = getCov4x4(w);
wCov = cov(w(1,:),w(2,:));
% wCov = getCov3x3(w);
wPi  = inv(wCov);


%% Select time frames
tFrame1 = [9.8,13.15];
[~,tStart1] = min(abs(t-tFrame1(1)));
[~,tEnd1] = min(abs(t-tFrame1(2)));
tS1 = t(tStart1:tEnd1);
tS1 = tS1 - tS1(1);
wS1 = w(:,tStart1:tEnd1);

tFrame2 = [17.8,19.125];
[~,tStart2] = min(abs(t-tFrame2(1)));
[~,tEnd2] = min(abs(t-tFrame2(2)));
tS2 = t(tStart2:tEnd2);
tS2 = tS2 - tS2(1);
wS2 = w(:,tStart2:tEnd2);


%% Estimate smoothness for unshifted input
% TODO Assume:
% - Gaussian filter validity
% - Same s for each state and output
wCovS1   = cov(wS1(1,:),wS1(2,:));
wPiS1    = inv(wCovS1);
s = sqrt(wPiS1(2,2)/(2*wPiS1(1,1)));

[~,sEst1] = estimateNoiseCharacteristics(tS1,wS1,1,1);


wCovS2   = cov(wS2(1,:),wS2(2,:));
wPiS2    = inv(wCovS2);
s = sqrt(wPiS2(2,2)/(2*wPiS2(1,1)));

[~,sEst2] = estimateNoiseCharacteristics(tS2,wS2,1,1);

% sEst2 = estimateSmoothness(t(1:end-1),w);

% wS      = [w(1,:);w(3,:);w(2,:);w(4,:)];
% wCovTS = getCov4x4(wS);
% wPiTS  = inv(wCovTS);

% s1 = sqrt(wPiTS(3,3)/(2*wPiS(1,1)));
% s2 = sqrt(wPiTS(3,4)/(2*wPiS(1,2)));
% s3 = sqrt(wPiTS(4,3)/(2*wPiS(2,1)));
% s4 = sqrt(wPiTS(4,4)/(2*wPiS(2,2)));


%% Estimate smoothness for forward shifted input
% TODO Assume:
% - Gaussian filter validity
% - Same s for each state and output
% Shift input sequence
[~,sStart] = min(abs(t-10.65));
[~,sEnd] = min(abs(t-10.8917));
nShift = sEnd - sStart;
tShift = t(nShift+1:end);
tShift = tShift - tShift(1);
xLinShift = xLin(:,nShift+1:end);
uLinShift = uLin(:,1:end-nShift);

% Calculate process noise
wShift = zeros(nx,length(tShift)-1);
for i = 1:length(tShift)-1
    wShift(:,i) = xLinShift(:,i+1) - sysD.A*xLinShift(:,i) - ...
                  sysD.B*uLinShift(:,i);
end

tFrame1 = [9.8,13.25];
[~,tSStart1] = min(abs(tShift-tFrame1(1)));
[~,tSEnd1] = min(abs(t-tFrame1(2)));
tSS1 = t(tSStart1:tSEnd1);
tSS1 = tSS1 - tSS1(1);
wSS1 = w(:,tSStart1:tSEnd1);
wShiftCovS1   = cov(wSS1(1,:),wSS1(2,:));
wShiftPiS1    = inv(wShiftCovS1);
[~,sEstShift1] = estimateNoiseCharacteristics(tSS1,wSS1,1,1);

tFrame2 = [17.8,19.2];
[~,tSStart2] = min(abs(tShift-tFrame2(1)));
[~,tSEnd2] = min(abs(t-tFrame2(2)));
tSS2 = t(tSStart2:tSEnd2);
tSS2 = tSS2 - tSS2(1);
wSS2 = w(:,tSStart2:tSEnd2);
wShiftCovS2   = cov(wSS2(1,:),wSS2(2,:));
wShiftPiS2    = inv(wShiftCovS2);
[~,sEstShift2] = estimateNoiseCharacteristics(tSS2,wSS2,1,1);
% sEst2 = estimateSmoothness(t(1:end-1),w);


%% Plot data
figure('Name','Compare shifted input');
subplot(2,1,1);
hold on;
plot(t,xLin(2,:));
plot(t,sysD.B(2,:)*uLin);
plot(t(1:end-1),w(2,:));
subplot(2,1,2);
hold on;
plot(tShift,xLinShift(2,:));
plot(tShift,sysD.B(2,:)*uLinShift);
plot(tShift(1:end-1),wShift(2,:));

figure('Name','Process noise roll rate');
hold on;
subplot(2,1,1);
plot(t(1:end-1),w(1,:));
subplot(2,1,2);
% plot(t,xLin(2,:));
% plot(t,sysD.B(2,:)*uLin);
plot(t(1:end-1),w(2,:));
yline(0);

figure('Name','Process noises in time frames');
subplot(2,2,1);
plot(tS1,wS1(1,:));
subplot(2,2,3);
plot(tS1,wS1(2,:));
subplot(2,2,2);
plot(tS2,wS2(1,:));
subplot(2,2,4);
plot(tS2,wS2(2,:));


figure('Name','Shifted process noise roll rate');
hold on;
subplot(2,1,1);
plot(tShift(1:end-1),wShift(1,:));
subplot(2,1,2);
% plot(tShift,xLinShift(2,:));
% plot(tShift,sysD.B(2,:)*uLinShift);
plot(tShift(1:end-1),wShift(2,:));
yline(0);

figure('Name','Shifted process noises in time frames');
subplot(2,2,1);
plot(tSS1,wSS1(1,:));
subplot(2,2,3);
plot(tSS1,wSS1(2,:));
subplot(2,2,2);
plot(tSS2,wSS2(1,:));
subplot(2,2,4);
plot(tSS2,wSS2(2,:));
