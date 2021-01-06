%% Initialization
clear;
close all;
clc;


%% Load error data
load SSE_VAF_0-7_0-7_1e-4-1e-4-1e-3_2e-3-1e-3-1.5e-2.mat;

% Select:
% p: 1-6
% d: 1-6
% s: 6e-4 - 1e-2
pSel = 2:1:7;
dSel = 2:1:7;
% sSel = 6:1:20;
sSel = 4:1:22;
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
sZ = 7.1e-3;
sW = [9.3e-3;3.8e-3];

lineLabelFontSize = 25;
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;

cP = [0,      0.4470, 0.7410;
      0.8500, 0.3250, 0.0980;
      0.9290, 0.6940, 0.1250;
      0.4940, 0.1840, 0.5560;
      0.4660, 0.6740, 0.1880;
      0.3010, 0.7450, 0.9330;
      0.6350, 0.0780, 0.1840];

figure('Name','DEM vs Kalman results for s values and p/d in range 1-6');
box on;
hold on;
errorbar(s_range(sSel),sseMean,sseMean-sseMin,sseMax-sseMean,...
         'CapSize',20,'Color',cP(1,:),'LineWidth',3);
yline(kalmanErr,'Color',cP(2,:),'FontSize',lineLabelFontSize,...
      'Label',['SSE Kalman: ' num2str(kalmanErr)],'LabelHorizontalAlignment','center','LineWidth',3);
xline(sZ,'Color',cP(3,:),'FontSize',lineLabelFontSize,...
      'Label',['s_z = ' num2str(sZ)],'LineWidth',2);
xline(sW(1),'Color',cP(4,:),'FontSize',lineLabelFontSize,...
      'Label',['s_{w_1} = ' num2str(sW(1))],'LineWidth',2);
xline(sW(2),'Color',cP(5,:),'FontSize',lineLabelFontSize,...
      'Label',['s_{w_{2,res}} = ' num2str(sW(2))],'LineWidth',2);
xlim([0,0.0135]);
ylim([0,40]);
legend('DEM','Kalman','s_z','s_{w_1}','s_{w_{2,res}}');
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
