%% Initialization
clear;
close all;
clc;


%% Select simulation data
% Load pre-processed simulation data file
load hoverSpiralling25-100Hz15-120s.mat expData;

% Select samples to use in simulation
startSample = 1;
endSample = 2600;

t      = expData.state.otTime(startSample:endSample);
pos    = expData.state.otPos(:,startSample:endSample);
orient = expData.state.otOrient(:,startSample:endSample);

uMotor = expData.input.motor(:,startSample:endSample);

dur    	= length(t);


% FFT of input data
uMotor = uMotor - mean(uMotor,2);
ts = t(2) - t(1);
fs = 1/ts;
fU = fft(uMotor,dur,2);
pU2 = abs(fU/dur);
pU1 = pU2(:,1:dur/2+1);
pU1(:,2:end-1) = 2*pU1(:,2:end-1);
f = fs*(0:(dur/2))/dur;
% plot(f,pU1);
% plot(t,uMotor(1,:));


%% System parameters
param.g             = 9.81;
param.l             = 0.178;
param.m             = 0.481;
param.PwmToPwm      = 2.55;
param.PwmToOmegaR   = [3.7,130.9];
param.cT            = [8.6e-6,-3.2e-4];
param.cQ            = [2.4e-7,-9.9e-6];


%% Possible filters
% Extracting peak envelope
nEnv = 3;
envHigh = zeros(4,dur);
envMean = zeros(4,dur);
envLow  = zeros(4,dur);
for i = 1:4
    [envHigh(i,:),envLow(i,:)] = envelope(uMotor(i,:),nEnv,'peak');
    envMean(i,:) = (envHigh(i,:)+envLow(i,:))/2;

%     figure('Name',['Motor ' num2str(i)]);
%     plot(t,uMotor(i,:),...
%          t,envHigh(i,:),...
%          t,envMean(i,:),'-o',...
%          t,envLow(i,:));
%      legend('Motor values','envHigh','envMean','envLow');
end
nCut = 2*nEnv;
tEnv    = t(nCut+1:end-nCut);
envHigh = envHigh(:,nCut+1:end-nCut);
envMean = envMean(:,nCut+1:end-nCut);
envLow  = envLow(:,nCut+1:end-nCut);

% for i = 1:4
%     figure('Name',['Motor ' num2str(i)]);
%     plot(t,uMotor(i,:),...
%          tEnv,envHigh(i,:),...
%          tEnv,envMean(i,:),'-o',...
%          tEnv,envLow(i,:));
%      legend('Motor values','envHigh','envMean','envLow');
% end


% Moving average filter
nAvg = 10;
coeff = ones(1,nAvg)/nAvg;
delay = (nAvg-1)/2;
movAvg = zeros(4,dur);
for i = 1:4
    movAvg(i,:) = filter(coeff,1,uMotor(i,:));
end
nCut = 0;
% figure('Name',['Moving average filter; average over ' num2str(nAvg) ...
%                ' samples']);
% plot(t,uMotor(1,:))
% hold on;
% plot(t-delay*ts,movAvg(1,:));


%% Simulation parameters
% Select input
uMotorFilt = movAvg;
dur = length(uMotorFilt);

% Construct input from rotor speeds
pwmToolbox  = uMotorFilt/param.PwmToPwm;
omegaR      = param.PwmToOmegaR(1)*pwmToolbox+param.PwmToOmegaR(2);
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

% Put input in format to use for system identification
u = u - [param.m*param.g;0;0;0];
u = u';

% figure('Name','T');
% plot(tFilt,u(:,1));
% figure('Name','tau_phi');
% plot(tFilt,u(:,2));
% figure('Name','tau_theta');
% plot(tFilt,u(:,3));
% figure('Name','tau_psi');
% plot(tFilt,u(:,4));


%% Construct output in format to use for system identification
y = [pos(:,nCut+1:end-nCut);orient(:,nCut+1:end-nCut)]';
y(:,3) = y(:,3) - 1;

yFilt = zeros(dur,6);
for i = 1:6
    yFilt(:,i) = filter(coeff,1,y(:,i));
end
% figure('Name',['Moving average filter; average over ' num2str(nAvg) ...
%                ' samples']);
% plot(t,y(:,i))
% hold on;
% plot(t-delay*ts,yFilt(:,i));

