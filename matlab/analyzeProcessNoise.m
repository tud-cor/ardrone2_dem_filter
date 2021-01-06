%% Initialization
clear;
close all;
clc;

% Parameters
degToRad = 2*pi/180; %conversion from degrees to radians
pwmEq    = 169.0612; %Parrot battery PWM equilibrium (from tests in lab)

% Changeable
% Model input definition
% 0: PWM values as input
% 1: using PWM values, derive torque around x-axis and use that as input to
% linear model
uSelect = 1;


%% System parameters
% Environmental constants
g      = 9.81;     %m/s^2
m      = 0.481;    %kg
ixx    = 3.4e-3;   %kgm^2
ixy    = 0;        %kgm^2
ixz    = 0;        %kgm^2
iyy    = 4.0e-3;   %kgm^2
iyz    = 0;        %kgm^2
izz    = 6.9e-3;   %kgm^2
l      = 0.178;	%m

% Thrust and torque coefficients
% omegaR = cM(1)*pwm_toolbox + cM(2)
cM = [3.7,130.9];

% omegaR = cA(1)*pwm_ardrone + cA(2)
cA = [cM(1)/2.55,cM(2)];

% cT(1)*omegaR^2 + cT(2)*omegaR
cTO   = [8.6e-6,-3.2e-4]; %Own work
% cTEs = [1.281e-5,-1.677e-3]; %Estimated from tests at home
                               %(ground and average hovering constraint)
cTEs  = [1.275e-5;-1.670e-3]; %Estimated from tests in lab
                              %(ground and average hovering constraint)
cTEs2 = [1.021e-5;-7.037e-4]; %Estimated from tests in lab
                              %(2 hovering constraints)

% cQ(1)*omegaR^2 + cQ(2)*omegaR
cQ = [2.4e-7,-9.9e-6];


% Choose coefficients
cT = cTEs2;

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
C = zeros(ny,nx);
C(1:3,1:3) = eye(3);
C(4:6,7:9) = eye(3);
D = zeros(ny,nu);


%% Determine process noise for completely linearized with PWM as input
B.u0 = [0,          0,          0,         0;
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


%% Determine process noise for input calculated based on inputs in
%  nonlinear way
B.u1 = [0, 0,     0,     0;
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


%% LTI state-space description and discretize - option 7
% Construct continuous-time linearised state space system
% (option 7 in notes: only phi,phiDot)
xSel = [7,10];
if ~uSelect
    uSel = 1:4;
else
    uSel = 2;
end

nx = length(xSel);
nu = length(uSel);
ny = 1;

A7 = A(xSel,xSel);
B7 = B(xSel,uSel);
C7 = [1,0];
D7 = zeros(ny,nu);

% Linearized system analysis
lambda7 = eig(A7);
con7    = ctrb(A7,B7);
nUncon7 = size(con7,1) - rank(con7);
obs7    = obsv(A7,C7);
nUnobs7 = size(obs7,2) - rank(obs7);


%% Construct input data
pwm = expData.input.navMotor;
if ~uSelect
    u = pwm;
    uOp = pwmEq;
else
    f = zeros(4,nDur);
    u = zeros(3,nDur); %ignoring yaw
    for i = 1:nDur
        f(:,i) = cT(1)*cA(1)^2*pwm(:,i).^2 + ...
                 (2*cT(1)*cA(1)*cA(2) + cT(2)*cA(1))*pwm(:,i) + ...
                 kron(ones(4,1),cT(1)*cA(2)^2 + cT(2)*cA(2));
        u(1,i) = f(1,i) + f(2,i) + f(3,i) + f(4,i);
        u(2,i) = sqrt(1/2)*l*(f(1,i)-f(2,i)-f(3,i)+f(4,i));
        u(3,i) = sqrt(1/2)*l*(-f(1,i)-f(2,i)+f(3,i)+f(4,i));
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
x = [phi;phiDot];

% Convert to state around operating point
xOp = [0;0];
xLin = x - xOp;

% Only measure yDot and phiDot
y = phi;

% Convert to output around operating point
yOp = 0;
yLin = y - yOp;


%% Select time frames
tFrame1 = [9.8,13.15];
tFrame2 = [17.8,19.125];
tFrame3 = [5.7,7.95];
tFrame4 = [9.7667,13.25];
tFrame = tFrame4;
[~,tStart] = min(abs(t-tFrame(1)));
[~,tEnd] = min(abs(t-tFrame(2)));

t = t(tStart:tEnd);
t = t - t(1);
yLin = yLin(:,tStart:tEnd);
xLin = xLin(:,tStart:tEnd);
uLin = uLin(:,tStart:tEnd);

f = f(:,tStart:tEnd);
pwm = pwm(:,tStart:tEnd);


%% Estimate process noise properties
% Discretize system
sysC = ss(A7,B7,C7,D7);
sysD = c2d(sysC,ts);

% Calculate process noise
tW = t(2:end);
w = zeros(nx,length(t)-1);
for i = 1:length(t)-1
    w(:,i) = xLin(:,i+1) - sysD.A*xLin(:,i) - sysD.B*uLin(:,i);
end

% Calculate precision matrix of process noise
% wCov = getCov4x4(w);
% wCov = getCov3x3(w);
wCov = cov(w(1,:),w(2,:));
wPi  = inv(wCov);


%% Estimate smoothness using fitted w2
% Generate fourier fit to process noise of roll rate
[w2Fit,w2FitGof,w2FitOut] = fit(tW',w(2,:)','fourier8');
w2FitRes = w2FitOut.residuals';

% [~,sEst1] = estimateProcessNoiseCharacteristics(t,[w(1,:);w2FitRes],1,1);
% sEst2 = estimateSmoothness(t(1:end-1),w);


%% Plot data of states, inputs and process noise
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;

% Generate simple derivative of roll rate measurement
xDer = diff(xLin,1,2);

% figure;
% subplot(2,1,1);


figure('Name','Process noise roll rate analysis');
subplot(3,1,1);
plot(tW,w(2,:));
subplot(3,1,2);
plot(tW,xDer(2,:));
hold on;
plot(t,sysD.B(2,:)*uLin);
subplot(3,1,3);
plot(t,pwm);
hold on;
yline(pwmEq);

% figure('Name','Process noise roll rate');
% hold on;
% plot(t,xLin(2,:));
% plot(t,sysD.B(2,:)*uLin);
% plot(t(1:end-1),w(2,:));
% yline(0);

% figure('Name','Process noise');
% box on;
% subplot(2,1,1);
% plot(tW,w(1,:));
% xlabel('Time (s)','FontSize',labelFontSize);
% ylabel('w_1 (rad)','FontSize',labelFontSize);
% title('Process noise','FontSize',titleFontSize);
% ax = gca;
% ax.FontSize = axFontSize;
% box on;
% subplot(2,1,2);
% plot(tW,w(2,:));
% xlabel('Time (s)','FontSize',labelFontSize);
% ylabel('w_2 (rad/s)','FontSize',labelFontSize);
% ax = gca;
% ax.FontSize = axFontSize;
