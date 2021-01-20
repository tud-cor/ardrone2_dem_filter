%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot impact of generalized coordinates
%
% Script to compare the DEM filter results with and without generalized
% states, outputs and inputs.
% 
% Author:        Dennis Benders, TU Delft, CoR
% Last modified: 20.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear;
close all;
clc;


%% Settings for plotting
% Define font sizes
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;

% Define plotting options
lineStyles = {'-','--','-.',':'};
lineWidth = 3;


%% Load data as reference using generalized states, inputs and outputs
load GenCoordsResults.mat;
xEstGenCoords   = xEst;
sseGenCoordsObs = sse.xobs;
sseGenCoordsHid = sse.xh;
clear xEst sse;


%% Load data to show usefulness of generalized states
load NoGenStatesResults.mat;
xEstNoGenStates   = xEst(:,1:end-2);
sseNoGenStatesObs = sse.xobs;
sseNoGenStatesHid = sse.xh;
clear xEst sse;

figure;
subplot(2,1,1);
box on;
hold on;
plot(t,x(1,:),'LineStyle',lineStyles{1},'LineWidth',lineWidth);
plot(t,xEstNoGenStates(1,:),'LineStyle',lineStyles{2},...
     'LineWidth',lineWidth);
plot(t,xEstGenCoords(1,:),'LineStyle',lineStyles{3},'LineWidth',lineWidth);
ylim([-0.3,0.2]);
legend('Measured',['DEM without gen. states (SSE = ' ...
                         num2str(sseNoGenStatesObs,3) ')'],...
                        ['DEM with gen. states (SSE = ' ...
                         num2str(sseGenCoordsObs,3) ')']);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\phi$ (rad)','FontSize',labelFontSize,'Interpreter','latex');
title('DEM roll angle estimates with and without generalized states',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(2,1,2);
box on;
hold on;
plot(t,x(2,:),'LineStyle',lineStyles{1},'LineWidth',lineWidth);
plot(t,xEstNoGenStates(2,:),'LineStyle',lineStyles{2},...
     'LineWidth',lineWidth);
plot(t,xEstGenCoords(2,:),'LineStyle',lineStyles{3},'LineWidth',lineWidth);
ylim([-1.3,1.7]);
legend('Measured',['DEM without gen. states (SSE = ' ...
                         num2str(sseNoGenStatesHid,3) ')'],...
                        ['DEM with gen. states (SSE = ' ...
                         num2str(sseGenCoordsHid,3) ')']);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\dot{\phi}$ (rad/s)','FontSize',labelFontSize,...
                              'Interpreter','latex');
title('DEM roll rate estimates with and without generalized states',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;


%% Load data to show usefulness of generalized output
load NoGenOutputsResults.mat;
xEstNoGenOutputs   = xEst;
sseNoGenOutputsObs = sse.xobs;
sseNoGenOutputsHid = sse.xh;
clear xEst sse;

figure;
subplot(2,1,1);
box on;
hold on;
plot(t,x(1,:),'LineStyle',lineStyles{1},'LineWidth',lineWidth);
plot(t,xEstNoGenOutputs(1,:),'LineStyle',lineStyles{2},...
     'LineWidth',lineWidth);
plot(t,xEstGenCoords(1,:),'LineStyle',lineStyles{3},'LineWidth',lineWidth);
ylim([-0.3,0.2]);
legend('Measured',['DEM without gen. output (SSE = ' ...
                         num2str(sseNoGenOutputsObs,3) ')'],...
                         ['DEM with gen. output (SSE = ' ...
                          num2str(sseGenCoordsObs,3) ')']);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\phi$ (rad)','FontSize',labelFontSize,'Interpreter','latex');
title('DEM roll angle estimates with and without generalized output',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(2,1,2);
box on;
hold on;
plot(t,x(2,:),'LineStyle',lineStyles{1},'LineWidth',lineWidth);
plot(t,xEstNoGenOutputs(2,:),'LineStyle',lineStyles{2},...
     'LineWidth',lineWidth);
plot(t,xEstGenCoords(2,:),'LineStyle',lineStyles{3},'LineWidth',lineWidth);
ylim([-1.3,1.7]);
legend('Measured',['DEM without gen. output (SSE = ' ...
                         num2str(sseNoGenOutputsHid,3) ')'],...
                         ['DEM with gen. output (SSE = ' ...
                          num2str(sseGenCoordsHid,3) ')']);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\dot{\phi}$ (rad/s)','FontSize',labelFontSize,...
                              'Interpreter','latex');
title('DEM roll rate estimates with and without generalized output',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;


%% Load data to show usefulness of generalized inputs
load NoGenInputsResults.mat;
xEstNoGenInputs   = xEst;
sseNoGenInputsObs = sse.xobs;
sseNoGenInputsHid = sse.xh;
clear xEst sse;

figure;
subplot(2,1,1);
box on;
hold on;
plot(t,x(1,:),'LineStyle',lineStyles{1},'LineWidth',lineWidth);
plot(t,xEstNoGenInputs(1,:),'LineStyle',lineStyles{2},...
     'LineWidth',lineWidth);
plot(t,xEstGenCoords(1,:),'LineStyle',lineStyles{3},'LineWidth',lineWidth);
ylim([-0.3,0.2]);
legend('Measured',['DEM without gen. inputs (SSE = ' ...
                         num2str(sseNoGenInputsObs,3) ')'],...
                         ['DEM with gen. inputs (SSE = ' ...
                          num2str(sseGenCoordsObs,3) ')']);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\phi$ (rad)','FontSize',labelFontSize,'Interpreter','latex');
title('DEM roll angle estimates with and without generalized inputs',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(2,1,2);
box on;
hold on;
plot(t,x(2,:),'LineStyle',lineStyles{1},'LineWidth',lineWidth);
plot(t,xEstNoGenInputs(2,:),'LineStyle',lineStyles{2},...
     'LineWidth',lineWidth);
plot(t,xEstGenCoords(2,:),'LineStyle',lineStyles{3},'LineWidth',lineWidth);
ylim([-1.3,1.7]);
legend('Measured',['DEM without gen. inputs (SSE = ' ...
                         num2str(sseNoGenInputsHid,3) ')'],...
                         ['DEM with gen. inputs (SSE = ' ...
                          num2str(sseGenCoordsHid,3) ')']);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\dot{\phi}$ (rad/s)','FontSize',labelFontSize,...
                              'Interpreter','latex');
title('DEM roll rate estimates with and without generalized inputs',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;


%% Compare 1st-order generalized state (phi') with measured and estimated
%  phiDot
load GenCoordsResults.mat;
phiDot  = x(2,:);
phiDotEst = xEst(4,:);
phiDash = xEst(5,:);

figure;
box on;
hold on;
plot(t,phiDot,'LineStyle',lineStyles{1},'LineWidth',lineWidth);
plot(t,phiDotEst,'LineStyle',lineStyles{2},'LineWidth',lineWidth);
plot(t,phiDash,'LineStyle',lineStyles{3},'LineWidth',lineWidth);
ylim([-0.3,0.2]);
legend('Measured','DEM estimate','DEM derivative estimate');
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\dot{\phi}$ (rad/s)','FontSize',labelFontSize,...
       'Interpreter','latex');
title('DEM roll rate estimates','FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
