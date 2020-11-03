%% Initialization
clear;
close all;
clc;


%% Calculate PWM at equilibrium using determined coefficients
% Parameters
g = 9.81;
% m = 0.481;
m = 0.497;
cM = [3.7,130.9];
cA = [cM(1)/2.55,130.9];
cT = [8.6e-06,-3.2e-4];
cQ = [2.4e-7,-9.9e-6];

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
% % Name of rosbag in ./ros
% bagname = 'ardrone2_exp_2020-11-02_hover_batA1.bag';
% 
% % Time interval with respect to start of rosbag recording
% time = [8,52];
% 
% % Topic selection
% topics.cmdVel = 0;
% topics.modelInput = 0;
% topics.gazeboModelStates = 0;
% topics.rotorsMotorSpeed = 0;
% topics.optitrack = 0;
% topics.ardroneImu = 0;
% topics.ardroneNav = 1;
% topics.ardroneOdom = 0;
% 
% 
% % Get bag data
% % Retrieve bag file
% cd ~/.ros;
% bag = rosbag(bagname);
% cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;
% 
% topicsOut = storeBagdata(bag,topics,time);
% 
% if topics.ardroneNav
%     ardroneNavStampTime  = topicsOut.ardroneNav.stampTime;
%     ardroneNavRecordTime = topicsOut.ardroneNav.recordTime;
%     ardroneNavMotor      = topicsOut.ardroneNav.motor;
%     ardroneNavAltd       = topicsOut.ardroneNav.altd;
%     ardroneNavVLin       = topicsOut.ardroneNav.vLin;
%     ardroneNavALin       = topicsOut.ardroneNav.aLin;
%     ardroneNavRot        = topicsOut.ardroneNav.rot;
% end
% t = ardroneNavStampTime - ardroneNavStampTime(1);
% 
% % Select time interval
% figure;
% subplot(2,1,1);
% plot(t,ardroneNavAltd);
% subplot(2,1,2);
% plot(t,ardroneNavMotor);
% 
% % [~,navStart] = min(abs(t-17.4784));
% % [~,navEnd] = min(abs(t-46.8207));
% 
% [~,navStart] = min(abs(t-6.7616));
% [~,navEnd] = min(abs(t-35.4924));
% 
% save('hoverBatA1.mat','t','ardroneNavMotor','ardroneNavAltd','navStart','navEnd');


% load hoverBatP1.mat;
load hoverBatA1.mat;


% Calculate average PWM
pwmAvgs = mean(ardroneNavMotor(:,navStart:navEnd),2);
pwmAvg = mean(pwmAvgs)