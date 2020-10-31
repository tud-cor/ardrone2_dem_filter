function y = quat2EulAndWrap(x,doPlot)
% Set threshold to make Euler angles 0
eulThres = 6;

% Convert quaternions to Euler angles
x = x';
orient = zeros(size(x,1),3);
for i = 1:size(x,1)
    orient(i,:) = quat2eul(x(i,:),'ZYX');
end
orient = orient';

orientUnwrapped = unwrap(orient,eulThres,2);

% % Remove jumps of 2*pi in angle data and ensure the angles are centered
% % around 0 rad
y = unwrap(orient(1:2,:),eulThres,2);
for i = 1:2
    if mean(y(i,:)) > eulThres/2
        y(i,:) = y(i,:) - pi;
    elseif mean(y(i,:)) < -eulThres/2
        y(i,:) = y(i,:) + pi;
    end
end
y(3,:) = orient(3,:);

if doPlot
    figure('Name','Orientation in ZYX Euler angles');
    subplot(3,1,1);
    plot(orient(1,:));
    hold on;
    plot(orient(2,:));
    plot(orient(3,:));
    yline(0);
    legend('phi','theta','psi','0 ref','FontSize',20);
    title('quat2eul','FontSize',30);
    xlabel('Time (s)','FontSize',25);
    ylabel('Angle (rad)','FontSize',25);

    subplot(3,1,2);
    plot(orientUnwrapped(1,:));
    hold on;
    plot(orientUnwrapped(2,:));
    plot(orientUnwrapped(3,:));
    yline(0);
    legend('phi','theta','psi','0 ref','FontSize',20);
    title('quat2eul and unwrap','FontSize',30);
    xlabel('Time (s)','FontSize',25);
    ylabel('Angle (rad)','FontSize',25);

    subplot(3,1,3);
    plot(y(1,:));
    hold on;
    plot(y(2,:));
    plot(y(3,:));
    yline(0);
    legend('phi','theta','psi','0 ref','FontSize',20);
    title(['quat2EulAndWrap result: quat2eul, unwrap and add +/-\pi '...
           'for \phi and \theta'],'FontSize',30);
    xlabel('Time (s)','FontSize',25);
    ylabel('Angle (rad)','FontSize',25);
end
end