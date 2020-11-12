%% Initialization
clear;
close all;
clc;


%% Load error data
% load SSE_0-6_0-6_1e-3-1e-1.mat;
% load SSE_0-6_0-6_5e-3-2e-2.mat;
% load SSE_0-6_0-6_5e-3-1.5e-2.mat;
load SSE_0-6_0-6_1e-3-1.5e-2.mat;
% load SSE_wind2_tFrame2_0-6_0-6_1e-3-1e-3-1.5e-2.mat;
% load SSE_wind2_tFrame3_0-6_0-6_1e-3-1e-3-1.5e-2.mat;

% Ignore no generalized state (this always gives the same error)
dem_xt = SSE.DEMv_x(2:end,:,:);
dem_xh = SSE.DEMv_xh(2:end,:,:);
dem_xobs = SSE.DEMv_xobs(2:end,:,:);

n_p = size(dem_xt,1);
n_d = size(dem_xt,2);
n_s = size(dem_xt,3);


%% Determine index of minimum error
sseMinVec = zeros(n_s,1);
sMinIdxVec = zeros(n_s,1);
for k = 1:n_s
    [sseMinVec(k),sMinIdxVec(k)] = min(dem_xt(:,:,k),[],'all','linear');
end
[sseMin,sMin] = min(sseMinVec);
pMin = mod(sMinIdxVec(sMin),n_p);
dMin = ceil(sMinIdxVec(sMin)/n_p)-1;


%% Generate data to plot
% sSel = 0 plots the SSE for every combination of p, d and s
% sSel = 1 plots the SSE for every combination of p, d for the optimal s
sSel = 1;

x = kron(ones(n_p,1),0:1:6);
y = kron(ones(1,n_d),(1:1:6)');

if sSel
    surf(x,y,dem_xt(:,:,sMin));
    xlabel('x (d)');
    ylabel('y (p)');
    zlabel('z (SSE)');
else
    for k = 1:n_s
        figure;
        surf(x,y,dem_xt(:,:,k));
        xlabel('x (d)');
        ylabel('y (p)');
        zlabel('z (SSE)');
    end
end
