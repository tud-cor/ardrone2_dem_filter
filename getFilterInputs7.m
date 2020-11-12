%% Initialization
clear;
close all;
clc;

% Parameters
degToRad = 2*pi/180; %conversion from degrees to radians
% pwmEq    = 169.5916; %Parrot battery PWM equilibrium (from tests at home)
pwmEq    = 169.0612; %Parrot battery PWM equilibrium (from tests in lab)
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
m      = 0.481;    %kg
ixx    = 3.4e-3;   %kgm^2
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
% cTEs = [1.281e-05,-1.677e-3]; %Estimated from tests at home
cTEs = [1.275e-05;-1.670e-3]; %Estimated from tests in lab

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

% % Try PWM-thrust coefficients from Eindhoven thesis
% cTP  = [1.5618e-4, 1.0395e-2, 0.13894;
%         1.8150e-4, 8.7242e-3, 0.14425;
%         1.3478e-4, 7.3295e-3, 0.11698;
%         1.4306e-4, 5.7609e-3, 0.13362];
% cTPhiDer1 = cTP(1,1)*pwmEq + cTP(1,2);
% cTPhiDer2 = cTP(2,1)*pwmEq + cTP(2,2);
% cTPhiDer3 = cTP(3,1)*pwmEq + cTP(3,2);
% cTPhiDer4 = cTP(4,1)*pwmEq + cTP(4,2);
% bPhiDot1   = 1/ixx*cTPhiDer1;
% bPhiDot2   = 1/ixx*cTPhiDer2;
% bPhiDot3   = 1/ixx*cTPhiDer3;
% bPhiDot4   = 1/ixx*cTPhiDer4;

% Construct elements of B-matrix
bZDot     = 1/m*cTDer;
bPhiDot   = 1/ixx*cTPhiDer;
bThetaDot = 1/iyy*cTThetaDer;
bPsiDot   = 1/izz*cTPsiDer;


%% Load experiment data
load expData10_29_25_mod6.mat;
% load expData10_29_25_mod7_hover.mat; %hovering just before applying wind

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


%% LTI state-space description and discretize - option 7
% Construct continuous-time linearised state space system
% (option 7 in notes: only yDot,phi,phiDot)
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


%% Convert AR.Drone 2.0 on-board data to inertial frame EDIT: do not use!
% For each sample in AR.Drone 2.0 data:
% - Convert linear coordinates from body frame to inertial frame
% - Convert angular coordinates from body frame to inertial frame
navVLin = expData.output.navVLin;
imuALin = expData.output.imuALin;
imuVAng = expData.output.imuVAng;

nSamples = length(t);

navVLinI          = zeros(3,nSamples);
origImuALinOffset = expData.origData.imuALin(:,1) - [0;0;g];
imuALinComp       = zeros(3,nSamples);
imuALinComp2      = zeros(3,nSamples);
imuALinI          = zeros(3,nSamples);

imuVAngI  = zeros(3,nSamples);
odomVAngI = zeros(3,nSamples);

for i = 1:nSamples
% Get current orientation
psi   = expData.output.otOrient(1,i);
theta = expData.output.otOrient(2,i);
phi   = expData.output.otOrient(3,i);

% Construct homogeneous transformation matrix (translational dynamics)
% RBI: convert coordinates expressed in B to coordinates expressed in I
% RIB: convert coordinates expressed in I to coordinates expressed in B
RBI = eul2rotm([psi,theta,phi],'ZYX');
RIB = RBI';


% Construct rotation matrix for rotational dynamics
% RrBI: convert ZYX Euler angles expressed in B to ZYX Euler angles
%       expressed in I
RrBI = zeros(3,3);
RrBI(1,1) = 1;
RrBI(1,2) = sin(phi)*tan(theta);
RrBI(1,3) = cos(phi)*tan(theta);
RrBI(2,2) = cos(phi);
RrBI(2,3) = -sin(phi);
RrBI(3,2) = sin(phi)/cos(theta);
RrBI(3,3) = cos(phi)/cos(theta);


% Compensate for gravity effects in accelerometer data
% Assumption 1: no coriolis effects
% Assumption 2: IMU is placed with perfect orientation in drone
imuALinComp(:,i) = imuALin(:,1) - RIB*[0;0;g];
imuALinComp2(:,i) = imuALinComp(:,i) - origImuALinOffset;


% Express linear quantities in I
% TODO: check if velocity data is already in inertial frame - probably yes!
navVLinI(:,i) = rotz(rad2deg(psi))*navVLin(:,i);
imuALinI(:,i) = RBI*imuALinComp2(:,i);


% Express angular quantities in I
imuVAngI(:,i) = RrBI*imuVAng(:,i);
end


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


%% Estimate measurement noise properties
% Calculate precision matrix of outputs (assuming a very high precision)
zSigma = 4.5e-4;
zPi = eye(ny)/zSigma^2;


%% Estimate process noise properties
% Discretize system
sysC = ss(A7,B7,C7,D7);
sysD = c2d(sysC,ts);

% Calculate process noise
w = zeros(nx,length(t)-1);
for i = 1:length(t)-1
    w(:,i) = xLin(:,i+1) - sysD.A*xLin(:,i) - sysD.B*uLin(:,i);
end

% Calculate precision matrix of process noise
% wCov = getCov4x4(w);
% wCov = getCov3x3(w);
wCov = cov(w(1,:),w(2,:));
wPi  = inv(wCov);


%% Estimate smoothness
% TODO Assume:
% - Gaussian filter validity
% - Same s for each state and output

% [~,sEst1] = estimateNoiseCharacteristics(t,w,1,1);
% sEst2 = estimateSmoothness(t(1:end-1),w);


%% Plot data
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;
% figure('Name','States');
% subplot(2,1,1);
% plot(t,xLin(1,:));
% subplot(2,1,2);
% plot(t,xLin(2,:));
% 
% figure('Name','Roll rate and input');
% hold on;
% plot(t,xLin(2,:));
% plot(t,uLin);
% yline(0);
% 
% figure('Name','Process noise roll angle');
% hold on;
% plot(t,xLin(1,:));
% plot(t,sysD.B(1,:)*uLin);
% plot(t(1:end-1),w(1,:));
% yline(0);

% figure('Name','Process noise roll rate');
% hold on;
% plot(t,xLin(2,:));
% plot(t,sysD.B(2,:)*uLin);
% plot(t(1:end-1),w(2,:));
% yline(0);

% Generate fourier fit to process noise of roll rate
tW = t(2:end);
[wFit,fGof,fitOut] = fit(tW',w(2,:)','fourier7');

% Generate derivative of process noise data
n = length(tW);
wDer = zeros(1,n-1);
for i = 1:n-1
    wDer(i) = w(2,i+1) - w(2,i);
end

gausFitW = fitdist(fitOut.residuals,'Normal');
gausFitWDot = fitdist(wDer','Normal');

figure('Name','Process noise vs white noise');
box on;
subplot(3,1,1);
plot(tW,w(2,:));
hold on;
plot(wFit);
ax = gca;
ax.FontSize = axFontSize;
legend('Process noise of roll rate','Fitted Fourier series');
title('Process noise or roll rate and fitted Fourier series',...
      'FontSize',titleFontSize);
subplot(3,1,2);
plot(tW,fitOut.residuals);
title('Residuals after fit','FontSize',titleFontSize);
ax = gca;
xLim = ax.XLim;
yLim = ax.YLim;
ax.FontSize = axFontSize;
subplot(3,1,3);
plot(t(1:end-1),normrnd(gausFitW.mu,gausFitW.sigma,[1,length(t)-1]));
xlim(xLim);
ylim(yLim);
ax = gca;
ax.FontSize = axFontSize;
title('White noise with Guassian distribution',...
      'FontSize',titleFontSize);

figure('Name','Gaussian distribution of process noise and derivative');
box on;
xLim = [-0.17,0.17];
subplot(2,1,1);
histfit(fitOut.residuals,100,'normal');
xlim(xLim);
ax = gca;
ax.FontSize = axFontSize;
legend('Histogram of residuals','Gaussian fit');
title('Process noise','FontSize',titleFontSize);
subplot(2,1,2);
histfit(wDer,100,'normal');
xlim(xLim);
ax = gca;
ax.FontSize = axFontSize;
legend('Histogram of derivative of process noise','Gaussian fit',...
       'FontSize',labelFontSize);
title('Derivative of process noise','FontSize',titleFontSize);

% % TODO maybe conclude something about the DEM noise estimates
% load demNoiseEst.mat;
% figure('Name','Process noise of roll rate and estimate process noise by DEM');
% box on;
% hold on;
% plot(tW,fitOut.residuals);
% plot(tDash,wDash(2,:)*(ts*2));
% legend('Process noise of roll rate',...
%        'Process noise of roll rate, estimated by DEM');
% ax = gca;
% ax.FontSize = axFontSize;
% 
% % TODO maybe generate FFT of process noise and see what we can derive from
% % it frequencies
% [fW,pW] = getFFT(tW,w(2,:));
% figure;
% box on;
% plot(fW,pW);
% 
% [fR,pR] = getFFT(tW,fitOut.residuals');
% figure;
% box on;
% plot(fR,pR);


%% Save data
filename = sprintf('demCode/ardrone2FlightData7_%s',...
                   datestr(now,'dd-mm-yyyy_HH-MM'));

A = A7;
B = B7;
C = C7;

save(filename,...
     't','ts',...
     'uLin','xLin','yLin','wPi','zPi','s',...
     'A','B','C');
