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
xEstNoGenCoord = xEst(:,1:end-2);
sseNoGenCoord = sse.xh;

load GenCoordResults.mat;
xEstGenCoord = xEst;
sseGenCoord = sse.xh;

figure;
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
title('DEM estimates with and without generalized coordinates',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;


%% Load data to show usefulness of generalized output
load NoGenOutputResults.mat;
xEstNoGenOutput = xEst;
sseNoGenOutput = sse.xh;

load GenCoordResults.mat;
xEstGenOutput = xEst;
sseGenOutput = sse.xh;

figure;
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
title('DEM estimates with and without generalized output',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
