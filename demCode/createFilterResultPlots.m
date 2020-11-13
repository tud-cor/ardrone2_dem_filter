%% Initialization
clear;
close all;
clc;


%% Set fontsizes
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;


%% Load data to show usefulness of generalized coordinates
load NoGenCoordResults.mat;
xEstNoGenCoord = xEst(:,1:end-1);
sseNoGenCoordObs = sse.xobs;
sseNoGenCoord = sse.xh;

load GenCoordResults.mat;
xEstGenCoord = xEst;
sseGenCoordObs = sse.xobs;
sseGenCoord = sse.xh;

figure;
subplot(2,1,1);
box on;
hold on;
plot(t,x(1,:));
plot(t,xEstNoGenCoord(1,:));
plot(t,xEstGenCoord(1,:));
legend('Measured',['DEM without gen. coord. (SSE = ' ...
                         num2str(sseNoGenCoordObs) ')'],...
                        ['DEM with gen. coord. (SSE = ' ...
                         num2str(sseGenCoordObs) ')']);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\phi$ (rad)','FontSize',labelFontSize,...
                              'Interpreter','latex');
title('DEM roll angle estimates with and without generalized coordinates',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(2,1,2);
box on;
hold on;
plot(t,x(2,:));
plot(t,xEstNoGenCoord(2,:));
plot(t,xEstGenCoord(2,:));
legend('Measured',['DEM without gen. coord. (SSE = ' ...
                         num2str(sseNoGenCoord) ')'],...
                        ['DEM with gen. coord. (SSE = ' ...
                         num2str(sseGenCoord) ')']);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\dot{\phi}$ (rad/s)','FontSize',labelFontSize,...
                              'Interpreter','latex');
title('DEM roll rate estimates with and without generalized coordinates',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;


%% Load data to show usefulness of generalized output
load NoGenOutputResults.mat;
xEstNoGenOutput = xEst;
sseNoGenOutputObs = sse.xobs;
sseNoGenOutput = sse.xh;

load GenCoordResults.mat;
xEstGenOutput = xEst;
sseGenOutputObs = sse.xobs;
sseGenOutput = sse.xh;

figure;
subplot(2,1,1);
box on;
hold on;
plot(t,x(1,:));
plot(t,xEstNoGenOutput(1,:));
plot(t,xEstGenOutput(1,:));
legend('Measured',['DEM without gen. output (SSE = ' ...
                         num2str(sseNoGenOutputObs) ')'],...
                         ['DEM with gen. output (SSE = ' ...
                          num2str(sseGenOutputObs) ')']);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\phi$ (rad)','FontSize',labelFontSize,...
                              'Interpreter','latex');
title('DEM roll angle estimates with and without generalized output',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(2,1,2);
box on;
hold on;
plot(t,x(2,:));
plot(t,xEstNoGenOutput(2,:));
plot(t,xEstGenOutput(2,:));
legend('Measured',['DEM without gen. output (SSE = ' ...
                         num2str(sseNoGenOutput) ')'],...
                         ['DEM with gen. output (SSE = ' ...
                          num2str(sseGenOutput) ')']);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('$\dot{\phi}$ (rad/s)','FontSize',labelFontSize,...
                              'Interpreter','latex');
title('DEM roll rate estimates with and without generalized output',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
