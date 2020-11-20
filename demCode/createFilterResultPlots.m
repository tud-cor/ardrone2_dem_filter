%% Initialization
clear;
close all;
clc;


%% Set font sizes
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;


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
plot(t,x(1,:));
plot(t,xEstNoGenStates(1,:));
plot(t,xEstGenCoords(1,:));
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
plot(t,x(2,:));
plot(t,xEstNoGenStates(2,:));
plot(t,xEstGenCoords(2,:));
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
plot(t,x(1,:));
plot(t,xEstNoGenOutputs(1,:));
plot(t,xEstGenCoords(1,:));
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
plot(t,x(2,:));
plot(t,xEstNoGenOutputs(2,:));
plot(t,xEstGenCoords(2,:));
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
plot(t,x(1,:));
plot(t,xEstNoGenInputs(1,:));
plot(t,xEstGenCoords(1,:));
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
plot(t,x(2,:));
plot(t,xEstNoGenInputs(2,:));
plot(t,xEstGenCoords(2,:));
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
