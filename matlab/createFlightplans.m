%% Initialisation
clc;
clear;
close all;


%% Parameters
plot = 1;
constructFiles = 1;


%% Circle
% Generate data
n = 41;
t = linspace(0,20,n)';
f = 1/20;
omega = 2*pi*f;

a = 0.5;
zMin = 0.5;
x = rmvFPInacc(-a*sin(omega*t));
y = rmvFPInacc(a*cos(omega*t));
z = zMin*ones(n,1);
psi = zeros(n,1);
trajCircle = [x,y,z,psi];

% Plot
if (plot)
    figure('Name','Circle setpoints');
    scatter3(x,y,z,'o');
    pos1 = get(gcf,'Position');
    set(gcf,'Position',pos1-[pos1(3)/2,0,0,0]);
end

% Create text file
if (constructFiles)
    filename = ['circle_' num2str(2*a) 'x' num2str(2*a) 'm_n' ...
                num2str(n) '_zMin' num2str(zMin) '.txt'];
    constructTxtFile(filename,n,x,y,z,psi);
end


%% Inclined circle
% Generate z data
zInc = rmvFPInacc(-y+zMin+max(y));

% Plot
if (plot)
    figure('Name','Inclined circle setpoints');
    scatter3(x,y,zInc,'o');
    set(gcf,'Position',[pos1(1)+pos1(3)/2,pos1(2),pos1(3),pos1(4)]);
end

% Write to text file
if (constructFiles)
    filename = ['incCircle_' num2str(2*a) 'x' num2str(2*a) 'x' ...
                num2str(max(zInc)-min(zInc)) 'm_n' num2str(n) '_zMin' ...
                num2str(zMin) '.txt'];
    constructTxtFile(filename,n,x,y,zInc,psi);
end


%% 8-shape (lemniscate of Gerono)
% Generate data
n = 53;
t = linspace(0,50,n)';
f = 1/50;
omega = 2*pi*f;

a = 0.5;
zMin = 0.5;
x = rmvFPInacc(2*a*sin(omega*t).*cos(omega*t));
y = rmvFPInacc(a*sin(omega*t));
z = zMin*ones(n,1);
psi = zeros(n,1);
traj8 = [x,y,z,psi];

% Plot
if (plot)
    figure('Name','8-shape setpoints');
    scatter3(x,y,z,'o');
    set(gcf,'Position',[pos1(1)-pos1(3)/2,pos1(2)-pos1(4)-83,pos1(3),...
                        pos1(4)]);
end

% Write to text file
if (constructFiles)
    filename = ['8Shape_' num2str(2*a) 'x' num2str(2*a) 'm_n' ...
                num2str(n) '_zMin' num2str(zMin) '.txt'];
    constructTxtFile(filename,n,x,y,z,psi);
end


%% Inclined 8-shape
% Generate z-data
zInc = rmvFPInacc(-y+zMin+max(y));

% Plot
if (plot)
    figure('Name','Inclined 8-shape setpoints');
    scatter3(x,y,zInc,'o');
    set(gcf,'Position',[pos1(1)+pos1(3)/2,pos1(2)-pos1(4)-83,pos1(3),...
                        pos1(4)]);
end

% Write to text file
if (constructFiles)
    filename = ['inc8Shape_' num2str(2*a) 'x' num2str(2*a) 'x' ...
                num2str(max(zInc)-min(zInc)) 'm_n' num2str(n) '_zMin' ...
                num2str(zMin) '.txt'];
    constructTxtFile(filename,n,x,y,zInc,psi);
end


%% Function definitions
function output = rmvFPInacc(input)
thres = 1e-10;

output = input;
for i = 1:length(output)
    if (abs(output(i)) < thres)
        output(i) = 0;
    end
end
end


function constructTxtFile(filename,n,x,y,z,psi)
initialReachDist = 0.1;
stayWithinDist = 0.1;
stayTime = 0.1;
fileStart = ['setInitialReachDist ' num2str(initialReachDist) '\n'...
            'setStayWithinDist ' num2str(stayWithinDist) '\n'...
            'setStayTime ' num2str(stayTime) '\n\n'...
            'takeoff\n\n'...
            'goto 0 0 ' num2str(min(z)) ' 0\n\n'];
fileClose = ['\n'...
             'goto 0 0 ' num2str(min(z)) ' 0\n\n'...
             'land'];

fid = fopen(filename,'wt');
fprintf(fid,fileStart);
for i = 1:n
    stringToWrite = ['goto ' num2str(x(i)) ' ' num2str(y(i)) ' ' ...
                     num2str(z(i)) ' ' num2str(psi(i)) '\n'];
    fprintf(fid,stringToWrite);
end
fprintf(fid,fileClose);
fclose(fid);
end