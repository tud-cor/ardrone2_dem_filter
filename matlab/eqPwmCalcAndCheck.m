%% Initialization
clear;
close all;
clc;


%% Calculate PWM at equilibrium using determined coefficients
% Parameters
g = 9.81;
m = 0.481;
% m = 0.497;
cM = [3.7,130.9];
cA = [cM(1)/2.55,130.9];
cTO = [8.6e-06,-3.2e-4];
cTEs = [1.28e-05,-1.68e-3];
cQ = [2.4e-7,-9.9e-6];

% Select coefficients
cT = cTEs;

% Solve quadratic formula
a = 4*cT(1)*cA(1)^2;
b = 4*(2*cT(1)*cA(1)*cA(2) + cT(2)*cA(1));
c = 4*(cT(1)*cA(2)^2 + cT(2)*cA(2)) - m*g;
pwmE1 = (-b+sqrt(b^2-4*a*c))/(2*a);
pwmE2 = (-b-sqrt(b^2-4*a*c))/(2*a);

% Using roots
p = [a,b,c];
roots(p)


%% Experimentally obtain PWM at equilibrium
% Name of rosbag in ./ros
bagname = 'ardrone2_exp_hoverP2_10.bag';

% Time interval with respect to start of rosbag recording
% time = [10,32]; %hoverP1_1
% time = [21,27]; %hoverP1_2
% time = [16,33]; %hoverP1_4
% time = [13,40]; %hoverP2_6
% time = [14,40]; %hoverP2_7
% time = [15,29]; %hoverP2_9
time = [11,32]; %hoverP2_10

% time = [17,35]; %hoverA1_1
% time = [14,28]; %hoverA1_2
% time = [10,24]; %hoverA1_3
% time = [17,40]; %hoverA1_4
% time = [13,31]; %hoverA1_6
% time = [16,30]; %hoverA1_8
% time = [18,32]; %hoverA1_10

% Topic selection
topics.cmdVel = 0;
topics.modelInput = 0;
topics.gazeboModelStates = 0;
topics.rotorsMotorSpeed = 0;
topics.optitrack = 0;
topics.ardroneImu = 0;
topics.ardroneNav = 1;
topics.ardroneOdom = 0;


% Get bag data
% Retrieve bag file
cd ~/.ros;
bag = rosbag(bagname);
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

topicsOut = storeBagdata(bag,topics,time);

if topics.ardroneNav
    ardroneNavStampTime  = topicsOut.ardroneNav.stampTime;
    ardroneNavRecordTime = topicsOut.ardroneNav.recordTime;
    ardroneNavMotor      = topicsOut.ardroneNav.motor;
    ardroneNavAltd       = topicsOut.ardroneNav.altd;
    ardroneNavVLin       = topicsOut.ardroneNav.vLin;
    ardroneNavALin       = topicsOut.ardroneNav.aLin;
    ardroneNavRot        = topicsOut.ardroneNav.rot;
end
t = ardroneNavStampTime - ardroneNavStampTime(1);

% Select time interval
figure;
subplot(2,1,1);
plot(t,ardroneNavAltd);
subplot(2,1,2);
plot(t,ardroneNavMotor);
keyboard;

% Select data samples to use
prompt      = {'Enter time of 1st data sample:',...
               'Enter time of last data sample:'};
dlgtitle    = 'Data selection';
dims        = [1 35];
definput    = {num2str(t(1)),num2str(t(end))};
answer      = inputdlg(prompt,dlgtitle,dims,definput);
startTime   = round(str2double(answer{1}));
endTime     = round(str2double(answer{2}));
[~,navStart] = min(abs(t-startTime));
[~,navEnd]   = min(abs(t-endTime));

% [~,navStart] = min(abs(t-17.4784));
% [~,navEnd] = min(abs(t-46.8207));
% 
% [~,navStart] = min(abs(t-6.7616));
% [~,navEnd] = min(abs(t-35.4924));


% load hoverBatP1.mat;
% load hoverBatA1.mat;


% Calculate average PWM
pwmAvgs10 = mean(ardroneNavMotor(:,navStart:navEnd),2);
pwmAvg10 = mean(pwmAvgs10)


%% Save data
filename = 'hoverBatP.mat';
% save(filename,'pwmAvgs1','pwmAvg1');
save(filename,'pwmAvgs10','pwmAvg10','-append');
% save(filename,'time','t','ardroneNavAltd','ardroneNavMotor',...
%               'navStart','navEnd');
% save(filename,'time,','t','ardroneNavAltd','ardroneNavMotor',...
%               'navStart','navEnd','-append');
