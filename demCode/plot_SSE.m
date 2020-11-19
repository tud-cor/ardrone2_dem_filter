%% Initialization
clear;
close all;
clc;


%% Load error data
% load SSE_0-6_0-6_1e-3-1e-1.mat;
% load SSE_0-6_0-6_5e-3-2e-2.mat;
% load SSE_0-6_0-6_5e-3-1.5e-1.mat;
% load SSE_0-6_0-6_1e-3-1.5e-2.mat;
% load SSE_wind2_tFrame2_0-6_0-6_1e-3-1e-3-1.5e-2.mat;
% load SSE_wind2_tFrame3_0-6_0-6_1e-3-1e-3-1.5e-2.mat;
load SSE_VAF_0-7_0-7_1e-4-1e-4-1e-3_2e-3-1e-3-1.5e-2.mat;

% Select:
% p: 1-6
% d: 1-6
% s: 6e-4 - 1e-2
pSel = 2:1:7;
dSel = 2:1:7;
sSel = 6:1:20;
demErr = SSE.DEMv_x(pSel,dSel,sSel);
% dem_xh = SSE.DEMv_xh(2:end,:,:);
% dem_xobs = SSE.DEMv_xobs(2:end,:,:);

kalmanErr = 12.7;

nS = size(demErr,3);


%% Generate data to plot
% Determine average for each s value
sseMean = zeros(nS,1);
for k = 1:nS
    sseMean(k) = mean(demErr(:,:,k),'all');
end

% Determine minima for each s value
sseMin = zeros(nS,1);
sMinIdx = zeros(nS,1);
for k = 1:nS
    [sseMin(k),sMinIdx(k)] = min(demErr(:,:,k),[],'all','linear');
end

% Determine maxima for each s value
sseMax = zeros(nS,1);
sMaxIdx = zeros(nS,1);
for k = 1:nS
    [sseMax(k),sMaxIdx(k)] = max(demErr(:,:,k),[],'all','linear');
end


%% Plot SSE graph with errorbar for different s values (and p/d: 1-6)
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;
figure('Name','DEM vs Kalman results for s values and p/d in range 1-6');
box on;
hold on;
errorbar(s_range(sSel),sseMean,sseMean-sseMin,sseMax-sseMean,'CapSize',20);
yline(kalmanErr);
xline(7.1e-3);
xline(9.3e-3);
xline(3.8e-3);
ylim([0,40]);
legend('DEM (mean,min/max)','Kalman','s_z','s_{w_1}','s_{w_2}');
xlabel('Smoothness value','FontSize',labelFontSize);
ylabel('SSE','FontSize',labelFontSize);
title('DEM vs Kalman SSE','FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;


%% Determine index of minimum error
% sseMinVec = zeros(n_s,1);
% sMinIdxVec = zeros(n_s,1);
% for k = 1:n_s
%     [sseMinVec(k),sMinIdxVec(k)] = min(dem_xt(:,:,k),[],'all','linear');
% end
% [sseMin,sMin] = min(sseMinVec);
% pMin = mod(sMinIdxVec(sMin),n_p);
% dMin = ceil(sMinIdxVec(sMin)/n_p)-1;


%% Plot SSE for p and d values for each s value
% % sSel = 0 plots the SSE for every combination of p, d and s
% % sSel = 1 plots the SSE for every combination of p, d for the optimal s
% sSel = 0;
% 
% x = kron(ones(n_p,1),0:1:6);
% y = kron(ones(1,n_d),(1:1:6)');
% 
% if sSel
%     c = dem_xt(:,:,sMin);
%     surf(x,y,dem_xt(:,:,sMin),c,'EdgeColor','interp','FaceColor','interp');
%     colorbar;
%     xlabel('x (d)');
%     ylabel('y (p)');
%     zlabel('z (SSE)');
% else
%     for k = 1:n_s
%         figure;
%         c = dem_xt(:,:,k);
%         surf(x,y,dem_xt(:,:,k),c,'FaceColor','interp');
%         colorbar;
%         caxis([0,150]);
%         xlabel('x (d)');
%         ylabel('y (p)');
%         zlabel('z (SSE)');
%     end
% end
