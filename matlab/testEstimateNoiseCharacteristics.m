%% Initialization
clear;
close all;
clc;

%% Generate test signal and compare result of estimateNoiseCharacteristics
%  with Sigma and s used to generate the original signal
% Set time properties
n = 1000;
ts = 0.0083;
time = zeros(1,n);
for i = 2:n
    time(i) = time(i-1) + ts;
end

% Set Sigma and s
nx    = 6;
mu    = zeros(nx,1);
sigma = 0.0001;
Sigma = kron(eye(nx),sigma);
sMin  = 0.004;
sStep = 0.001;
s     = (sMin:sStep:sMin+sStep*(nx-1))';

% 1. Generate white noise signals
rng(2);
xW = zeros(nx,n);
for i = 1:nx
    xW(i,:) = normrnd(mu(i),Sigma(i,i),[1,n]);
end

% 2. Convolve with a Gaussian filter with kernel width s
tau = linspace(-time(end),time(end),2*n-1);
ts = time(2) - time(1);
xC = zeros(nx,n);
for i = 1:nx
    h = sqrt(1/ts*s(i)*sqrt(pi))*exp(-tau.^2/(2*s(i)^2));
    xC(i,:) = conv(h,xW(i,:),'valid');
end

% 3. Estimate Sigma and s using estimateNoiseCharacteristics
idxEst = [1,2,3,4,5,6];
SigmaEst = zeros(nx);
sEst = zeros(nx,1);
[SigmaEst(idxEst,idxEst),sEst(idxEst,1)] = ...
    estimateNoiseCharacteristics(time,xC(idxEst,:),0,0);


%% Get Sigma and s using estimate_smoothness function
sEst2 = zeros(nx,1);
sEst2(idxEst,1) = estimateSmoothness(time,xC(idxEst,:));


%% Print results
fprintf('GAUSSIAN FILTER\n');
fprintf('Input    Output   Err perc.\n');
fprintf('-------- -------- ---------\n');
fprintf('sigma:\n');
nx = size(SigmaEst,1);
for i = idxEst
    fprintf('%f %f %f\n',Sigma(i,i),SigmaEst(i,i),...
                         abs(SigmaEst(i,i)-Sigma(i,i))/Sigma(i,i)*100);
end
fprintf('\n');
fprintf('s:\n');
for i = idxEst
    fprintf('%f %f %f\n',s(i),sEst(i),abs(sEst(i)-s(i))/s(i)*100);
end

fprintf('\n\n');

fprintf('PARTICLE SWARM AND FMINCON\n');
fprintf('Input    Output   Err perc.\n');
fprintf('-------- -------- ---------\n');
fprintf('s:\n');
for i = idxEst
    fprintf('%f %f %f\n',s(i),sEst2(i),abs(sEst2(i)-s(i))/s(i)*100);
end