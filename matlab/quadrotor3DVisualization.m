function quadrotor3DVisualization(sampleTime,state,plotTitle)
% t: time array
% x: {x,y,z} position in inertial frame
%    {psi,theta,phi} ZYX Euler angles

% Standard parameters
l = 0.178;  %rotor arm length
r = 0.10;   %rotor radius

% To draw circles
nCircleSamples = 100;
omegaT = linspace(0,2*pi,nCircleSamples);

% Set figure properties
figure('Name','Quadrotor visualization');
c = [];
r1 = [];
r2 = [];
r3 = [];
r4 = [];
r1g = [];
r2g = [];
r3g = [];
r4g = [];
r1a = [];
r2a = [];
r3a = [];
r4a = [];
r1r3 = [];
r2r4 = [];
r1r3g = [];
r2r4g = [];
maxAxisLength = 5;
xMin = min(state(1,:)) - l;
xMax = max(state(1,:)) + l;
yMin = min(state(2,:)) - l;
yMax = max(state(2,:)) + l;
zMin = min(state(3,:)) - l;
zMax = max(state(3,:)) + l;
plotDim = [xMin;xMax;yMin;yMax;zMin;zMax];
for i = 1:6
    if abs(plotDim(i)) < 1
        if rem(i,2) == 1
            plotDim(i) = -1;
        else
            plotDim(i) = 1;
        end
    end
end
if abs(plotDim(2) - plotDim(1)) < maxAxisLength && ...
   abs(plotDim(4) - plotDim(3)) < maxAxisLength && ...
   abs(plotDim(6) - plotDim(5)) < maxAxisLength
    xLim = [plotDim(1),plotDim(2)];
    yLim = [plotDim(3),plotDim(4)];
    plotDim(5) = 0;
    zLim = [plotDim(5),plotDim(6)];
end
plot3([0;1],[0;0],[0;0],'Color','b','LineWidth',3);
grid on;
hold on;
plot3([0;0],[0;1],[0;0],'Color','b','LineWidth',3);
plot3([0;0],[0;0],[0;1],'Color','b','LineWidth',3);
if nargin > 2
    title(plotTitle,'FontSize',40);
end

% Plot quadrotor on every time step
for t = 1:size(state,2)
    % Process inputs
    x = state(1,t);
    y = state(2,t);
    z = state(3,t);
    if size(state,1) < 12
        psi = state(4,t);
        theta = state(5,t);
        phi = state(6,t);
    else
        psi = state(7,t);
        theta = state(8,t);
        phi = state(9,t);
    end

    % Construct homogeneous transformation matrix
    R = zeros(4,4);
    R(1:3,1:3) = eul2rotm([psi,theta,phi]);
    R(1,4) = x;
    R(2,4) = y;
    R(3,4) = z;
    R(4,4) = 1;

    % Set coordinates of quadrotor in body frame
    centerPosB = [0;0;-0.03;1];
    rotor1PosB = centerPosB + [l/sqrt(2);-l/sqrt(2);0;0];
    rotor2PosB = centerPosB + [l/sqrt(2);l/sqrt(2);0;0];
    rotor3PosB = centerPosB + [-l/sqrt(2);l/sqrt(2);0;0];
    rotor4PosB = centerPosB + [-l/sqrt(2);-l/sqrt(2);0;0];
    rotor1AxisB = [rotor1PosB,rotor1PosB+[0;0;0.03;0]];
    rotor2AxisB = [rotor2PosB,rotor2PosB+[0;0;0.03;0]];
    rotor3AxisB = [rotor3PosB,rotor3PosB+[0;0;0.03;0]];
    rotor4AxisB = [rotor4PosB,rotor4PosB+[0;0;0.03;0]];
    rotor1B = [rotor1AxisB(1,2)+r*cos(omegaT);...
               rotor1AxisB(2,2)+r*sin(omegaT);...
               zeros(1,nCircleSamples);ones(1,nCircleSamples)];
    rotor2B = [rotor2AxisB(1,2)+r*cos(omegaT);...
               rotor2AxisB(2,2)+r*sin(omegaT);...
               zeros(1,nCircleSamples);ones(1,nCircleSamples)];
    rotor3B = [rotor3AxisB(1,2)+r*cos(omegaT);...
               rotor3AxisB(2,2)+r*sin(omegaT);...
               zeros(1,nCircleSamples);ones(1,nCircleSamples)];
    rotor4B = [rotor4AxisB(1,2)+r*cos(omegaT);...
               rotor4AxisB(2,2)+r*sin(omegaT);...
               zeros(1,nCircleSamples);ones(1,nCircleSamples)];

    % Convert coordinates of quadrotor to inertial frame
    centerPosI = R*centerPosB;
    rotor1PosI = R*rotor1PosB;
    rotor2PosI = R*rotor2PosB;
    rotor3PosI = R*rotor3PosB;
    rotor4PosI = R*rotor4PosB;
    rotor1AxisI = R*rotor1AxisB;
    rotor2AxisI = R*rotor2AxisB;
    rotor3AxisI = R*rotor3AxisB;
    rotor4AxisI = R*rotor4AxisB;
    rotor1I = R*rotor1B;
    rotor2I = R*rotor2B;
    rotor3I = R*rotor3B;
    rotor4I = R*rotor4B;

    % Plot quadrotor in inertial frame
    delete(c);
    delete(r1);
    delete(r2);
    delete(r3);
    delete(r4);
    delete(r1g);
    delete(r2g);
    delete(r3g);
    delete(r4g);
    delete(r1a);
    delete(r2a);
    delete(r3a);
    delete(r4a);
    delete(r1r3);
    delete(r2r4);
    delete(r1r3g);
    delete(r2r4g);

    if exist('xLim','var')
        xlim(xLim);
        ylim(yLim);
        zlim(zLim);
    else
        xlim([x-maxAxisLength/2,x+maxAxisLength/2]);
        ylim([y-maxAxisLength/2,y+maxAxisLength/2]);
        zlim([z-maxAxisLength/2,z+maxAxisLength/2]);
    end
    c = plot3(centerPosI(1),centerPosI(2),centerPosI(3),'o','Color','k');
    r1 = plot3(rotor1I(1,:),rotor1I(2,:),rotor1I(3,:),'-','Color','b');
    r2 = plot3(rotor2I(1,:),rotor2I(2,:),rotor2I(3,:),'-','Color','g');
    r3 = plot3(rotor3I(1,:),rotor3I(2,:),rotor3I(3,:),'-','Color','r');
    r4 = plot3(rotor4I(1,:),rotor4I(2,:),rotor4I(3,:),'-','Color','k');
    r1g = plot3(rotor1I(1,:),rotor1I(2,:),zeros(1,nCircleSamples),'-',...
                'Color',[17,17,17]/255,'LineWidth',1);
    r2g = plot3(rotor2I(1,:),rotor2I(2,:),zeros(1,nCircleSamples),'-',...
                'Color',[17,17,17]/255,'LineWidth',1);
    r3g = plot3(rotor3I(1,:),rotor3I(2,:),zeros(1,nCircleSamples),'-',...
                'Color',[17,17,17]/255,'LineWidth',1);
    r4g = plot3(rotor4I(1,:),rotor4I(2,:),zeros(1,nCircleSamples),'-',...
                'Color',[17,17,17]/255,'LineWidth',1);
    r1a = plot3(rotor1AxisI(1,:),rotor1AxisI(2,:),rotor1AxisI(3,:),'-',...
                'Color','k');
    r2a = plot3(rotor2AxisI(1,:),rotor2AxisI(2,:),rotor2AxisI(3,:),'-',...
                'Color','k');
    r3a = plot3(rotor3AxisI(1,:),rotor3AxisI(2,:),rotor3AxisI(3,:),'-',...
                'Color','k');
    r4a = plot3(rotor4AxisI(1,:),rotor4AxisI(2,:),rotor4AxisI(3,:),'-',...
                'Color','k');
    r1r3 = plot3([rotor1PosI(1);rotor3PosI(1)],...
                 [rotor1PosI(2);rotor3PosI(2)],...
                 [rotor1PosI(3);rotor3PosI(3)],'-','Color','k');
    r2r4 = plot3([rotor2PosI(1);rotor4PosI(1)],...
                 [rotor2PosI(2);rotor4PosI(2)],...
                 [rotor2PosI(3);rotor4PosI(3)],'-','Color','k');
    r1r3g = plot3([rotor1PosI(1);rotor3PosI(1)],...
                 [rotor1PosI(2);rotor3PosI(2)],...
                 [0;0],'-','Color',[17,17,17]/255);
    r2r4g = plot3([rotor2PosI(1);rotor4PosI(1)],...
                 [rotor2PosI(2);rotor4PosI(2)],...
                 [0;0],'-','Color',[17,17,17]/255);

    drawnow;
    pause(sampleTime);
end
end