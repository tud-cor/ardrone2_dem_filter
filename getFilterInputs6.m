%% Initialization
clear;
close all;
clc;

% Parameters
degToRad = 2*pi/180; %conversion from degrees to radians
pwmEq    = 169.5916; %Parrot battery PWM equilibrium
% pwmEq    = 171.4937; %Akku-King battery PWM equilibrium


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
irotor = 2.030e-5;	%kgm^2 TODO: value from Q. Li (2014)

% Dimensions
l      = 0.178;	%m

% Thrust and torque coefficients
% omegaR = cM(1)*pwm_toolbox + cM(2)
cM = [3.7,130.9];

% omegaR = cA(1)*pwm_ardrone + cA(2)
cA = [cM(1)/2.55,cM(2)];

% cT(1)*omegaR^2 + cT(2)*omegaR
cTO = [8.6e-6,-3.2e-4]; %Own work
cTE = [1.11052992314982e-05,-0.000764005119869328]; %Eindhoven thesis
cTEs = [1.28081830181367e-05,-0.00167659115707409]; %Estimated

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
xSel = [2,5,7,10];

nx = length(xSel);
nu = 4;
ny = nx;

A6 = A(xSel,xSel);
B6 = B(xSel,:);
C6 = eye(nx);
D6 = zeros(ny,nu);

% Linearized system analysis
lambda6 = eig(A6);
con6    = ctrb(A6,B6);
nUncon6 = size(con6,1) - rank(con6);
obs6    = obsv(A6,C6);
nUnobs6 = size(obs6,2) - rank(obs6);


%% Load experiment data
load expData10_29_25_mod6.mat;

t    = expData.output.time;
ts   = expData.sampleTime;
nDur = length(t);


%% Convert AR.Drone 2.0 on-board data to inertial frame
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


%% Construct input data
u = expData.input.navMotor;
% pwmToolbox  = rotorSpeed/param.PwmToPwm;
% omegaR      = param.PwmToOmegaR(1)*pwmToolbox+param.PwmToOmegaR(2);
% f           = zeros(4,1);
% tauTheta    = zeros(1,nDur);
% for i = 1:nDur
%     f	 = param.cT(1)*omegaR(:,i).^2 + param.cT(2)*omegaR(:,i);
%     tauTheta(i) = sqrt(1/2)*param.l*(-f(1)-f(2)+f(3)+f(4));
% end
% u = tauTheta;

% Convert to input around operating point (PWM when using Parrot battery)
uOp  = pwmEq;
uLin = u - uOp;


%% Construct ground truth output data
yPos   = expData.output.otPos(2,:);
yDot   = navVLinI(2,:);
phi    = expData.output.otOrient(3,:);
phiDot = imuVAngI(1,:);
y = [yPos;yDot;phi;phiDot];

% Convert to output around operating point
yOp = [mean(yPos);0;0;0];
yLin = y - yOp;

% Set state data equal to output data
xLin = yLin;


%% Plot state/output data
figure('Name','y and derivatives');
subplot(3,1,1);
plot(t,yLin(1,:));
subplot(3,1,2);
plot(t,yLin(2,:));
subplot(3,1,3);
plot(t,imuALinI(2,:));

figure('Name','phi and derivative');
subplot(2,1,1);
plot(t,yLin(3,:));
subplot(2,1,2);
plot(t,yLin(4,:));
subplot(2,1,1);

figure('Name','phiDot and model input');
subplot(2,1,1);
plot(t,yLin(4,:));
subplot(2,1,2);
plot(t,cTPhiDer*[1,-1,-1,1]*uLin);


%% Estimate measurement noise properties
% Calculate precision matrix of outputs (assuming a very high precision)
zSigma = 1e-6;
zPi = eye(ny)/zSigma^2;


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
wCov = getCov4x4(w);
wPi  = inv(wCov);


%% Plot process noise data
figure;
for i = 1:nx
    subplot(nx,1,i);
    plot(t,xLin(i,:));
    hold on;
    plot(t(1:end-1),w(i,:));
end


%% Estimate smoothness
% TODO Assume:
% - Gaussian filter validity
% - Same s for each state and output
wCovS   = cov(w(1,:),w(3,:));
wPiS    = inv(wCovS);

wS      = [w(1,:);w(3,:);w(2,:);w(4,:)];
wCovTS = getCov4x4(wS);
wPiTS  = inv(wCovTS);

s1 = sqrt(wPiTS(3,3)/(2*wPiS(1,1)));
s2 = sqrt(wPiTS(3,4)/(2*wPiS(1,2)));
s3 = sqrt(wPiTS(4,3)/(2*wPiS(2,1)));
s4 = sqrt(wPiTS(4,4)/(2*wPiS(2,2)));

% s = mean([s1,s2,s3,s4]);
s = 0.006;

% s2 = estimateSmoothness(t(1:end-1),w);


%% Save data
filename = sprintf('demCode/ardrone2FlightData6_%s',...
                   datestr(now,'dd-mm-yyyy_HH-MM'));

A = A6;
B = B6;
C = C6;

save(filename,...
     't','ts',...
     'uLin','xLin','yLin','wPi','zPi','s',...
     'A','B','C');
