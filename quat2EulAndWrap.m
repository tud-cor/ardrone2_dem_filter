function y = quat2EulAndWrap(x,doPlot)
% Set threshold to make Euler angles 0
eulThres = 6;

% Convert quaternions to Euler angles
x = x';
orient = zeros(size(x,1),3);
for i = 1:size(x,1)
    orient(i,:) = quat2eul(x(i,:));
end
orient = orient';

% Remove jumps of 2*pi in angle data and ensure the angles are centered
% around 0 rad
y = unwrap(orient,eulThres,2);
for i = 1:3
    if mean(y(i,:)) > eulThres/2
        y(i,:) = y(i,:) - pi;
    elseif mean(y(i,:)) < -eulThres/2
        y(i,:) = y(i,:) + pi;
    end
end

if doPlot
    figure('Name','ZYX Euler angles');
    subplot(3,1,1);
    plot(y(1,:),'-o');
    subplot(3,1,2);
    plot(y(2,:),'-o');
    subplot(3,1,3);
    plot(y(3,:),'-o');
end
end