%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analyze coloured noise properties
%
% Script to create coloured noise (by convoluting white noise with a
% Gaussian filter) and estimate its properties (using the Gaussian filter
% assumption).
% 
% Author:        Dennis Benders, TU Delft, CoR
% Last modified: 20.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear;
close all;
clc;


%% Generate coloured noise
% Set time properties
n = 1200;
fs = 120;
ts = 1/fs;
time = zeros(1,n);
for i = 2:n
    time(i) = time(i-1) + ts;
end

% Set Sigma and s
nx    = 1;
mu    = zeros(nx,1);
sigma = 2;
s     = 5e-2;

% 1. Generate white noise signals
rng(1);
xW = normrnd(mu,sigma,[1,n]);

% 2. Convolve with a Gaussian filter with kernel width s
tau = linspace(-time(end),time(end),2*n-1);
ts = time(2) - time(1);
h = sqrt(ts/(s*sqrt(pi)))*exp(-tau.^2/(2*s^2));
xC = conv(h,xW,'valid');

% 3. Estimate s using estimateNoiseCharacteristics
[~,sEst] = estimateNoiseCharacteristics(time,xC,1,1);


%% Print results
fprintf('Original s : %6.4f\n',s);
fprintf('Estimated s: %6.4f\n',sEst);


%% Plot results
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;

figure('Name','White noise and coloured noises');
subplot(2,1,1);
plot(time,xW);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('Noise value','FontSize',labelFontSize);
title('White noise','FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(2,1,2);
plot(time,xC);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('Noise value','FontSize',labelFontSize);
title('Coloured noise','FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
