%% Initialization
clear;
close all;
clc;

% For OptiTrack noise:
% - optitrackNoiseTest (from ardrone2_exp_2020-07-24_4.bag)


%% Set variables
% % Retrieve bag file
% cd ~/.ros;
% bag = rosbag("ardrone2_exp_2020-10-29_25_batP1.bag");
% cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;
% 
% % Select topics that need to be stored
% % Controller signal topics
% topics.cmdVel = 0;
% 
% % Simulation/flight data topics
% topics.modelInput = 0;
% topics.gazeboModelStates = 0;
% topics.optitrack = 1;
% topics.ardroneImu = 0;
% topics.ardroneNav = 0;
% topics.ardroneOdom = 0;
% topics.rotorsMotorSpeed = 0;
% 
% % Set time interval with respect to start of rosbag recording
% time = [0,10];
% 
% 
% %% Get data
% topicsOut = storeBagdata(bag, topics, time);
% 
% if topics.cmdVel
%     cmdVelTime = topicsOut.cmdVel.time;
%     cmdVelLin = topicsOut.cmdVel.lin;
%     cmdVelAng = topicsOut.cmdVel.ang;
% end
% 
% if topics.optitrack
%     optitrackStampTime = topicsOut.optitrack.stampTime;
%     optitrackRecordTime = topicsOut.optitrack.recordTime;
%     optitrackPos = topicsOut.optitrack.pos;
%     optitrackOrientQuat = topicsOut.optitrack.orient;
% end
% 
% if topics.ardroneImu
%     ardroneImuStampTime  = topicsOut.ardroneImu.stampTime;
%     ardroneImuRecordTime  = topicsOut.ardroneImu.recordTime;
%     ardroneImuOrientQuat = topicsOut.ardroneImu.orient;
%     ardroneImuVAng       = topicsOut.ardroneImu.vAng;
% end
% 
% if topics.ardroneNav
%     ardroneNavStampTime = topicsOut.ardroneNav.stampTime;
%     ardroneNavRecordTime = topicsOut.ardroneNav.recordTime;
%     ardroneNavMotor = topicsOut.ardroneNav.motor;
%     ardroneNavRot = topicsOut.ardroneNav.rot;
%     ardroneNavVLin = topicsOut.ardroneNav.vLin;
%     ardroneNavALin = topicsOut.ardroneNav.aLin;
% end
% 
% if topics.ardroneOdom
%     ardroneOdomStampTime = topicsOut.ardroneOdom.stampTime;
%     ardroneOdomRecordTime = topicsOut.ardroneOdom.recordTime;
%     ardroneOdomPos = topicsOut.ardroneOdom.pos;
%     ardroneOdomOrientQuat = topicsOut.ardroneOdom.orient;
%     ardroneOdomVLin = topicsOut.ardroneOdom.vLin;
%     ardroneOdomVAng = topicsOut.ardroneOdom.vAng;
% end
% 
% 
% %% Select suitable time frames
% % Plot data to search for time where quantities are constant
% optitrackPlotTime = optitrackStampTime - optitrackStampTime(1);
% figure('Name','OptiTrack position data');
% hold on;
% plot(optitrackPlotTime,optitrackPos(1,:),'-o');
% plot(optitrackPlotTime,optitrackPos(2,:),'-o');
% plot(optitrackPlotTime,optitrackPos(3,:),'-o');
% keyboard;
% 
% % Select data samples to use
% prompt      = {'Enter time of 1st data sample:',...
%                'Enter time of last data sample:'};
% dlgtitle    = 'Data selection';
% dims        = [1 35];
% definput    = {num2str(optitrackPlotTime(1)),...
%                num2str(optitrackPlotTime(end))};
% answer      = inputdlg(prompt,dlgtitle,dims,definput);
% startTime   = str2double(answer{1}) + optitrackStampTime(1);
% endTime     = str2double(answer{2}) + optitrackStampTime(1);
% [~,otStart] = min(abs(optitrackStampTime-startTime));
% [~,otEnd]   = min(abs(optitrackStampTime-endTime));
% 
% % Select proper OptiTrack data
% optitrackStampTime = optitrackStampTime(otStart:otEnd);
% optitrackStampTime = optitrackStampTime - optitrackStampTime(1);
% optitrackPos = optitrackPos(:,otStart:otEnd);
% optitrackOrientQuat = optitrackOrientQuat(:,otStart:otEnd);
% 
% 
% %% Ensure that all orientations are given in ZYX Euler angles, start at 0
% %  and are given in [rad]
% 
% % Convert quaternion from ROS convention (x,y,z,w)
% %                    to MATLAB convention (w,x,y,z)
% optitrackOrientQuat = [optitrackOrientQuat(4,:);...
%                        optitrackOrientQuat(1:3,:)];
% 
% % Convert quaternions to ZYX Euler angles: [Z;Y;X] ([psi;theta;phi])
% optitrackOrient = quat2eul(optitrackOrientQuat','ZYX')';
% 
% figure('Name','OptiTrack orientation data');
% subplot(3,1,1);
% plot(optitrackStampTime,optitrackOrient(1,:),'-o');
% title('\psi');
% subplot(3,1,2);
% plot(optitrackStampTime,optitrackOrient(2,:),'-o');
% title('\theta');
% subplot(3,1,3);
% plot(optitrackStampTime,optitrackOrient(3,:),'-o');
% title('\phi');
% 
% 
% %% Interpolate data
% % Sample time
% fs = 120;
% measNoiseData.sampleTime = 1/fs;
% 
% % OptiTrack data
% data.time = optitrackStampTime;
% data.value = optitrackOrient(3,:);
% tmp = interpolate(measNoiseData.sampleTime,data);
% time = tmp.time;
% z = tmp.value;
% 
% 
% %% Ensure data start at time 0 and data is zeroed
% time = time - time(1);
% z = z - z(1);
% 
% 
% %% Save data to speed up
% % OptiTrack data
% save('optiTrackNoiseTest.mat','time','z');
% 
% 
%% Load data to speed up
% OptiTrack data
load optiTrackNoiseTest.mat;


%% Calculate noise characteristics of OptiTrack roll angle
%  states
% [f,pZ1] = getFFT(time,z);
% plot(f,pZ1);
% legend('x','y','z','\phi','\theta','\psi');
% ylim([0,0.3e-3]);

% for i = 1:ny
%     figure('Name',['Frequency spectrum of z' num2str(i)]);
%     plot(f,pZ1(i,:));
% end

% z = z - mean(z);
std(z)
std(z)^2

[SigmaZEst1,sZEstGaussian] = estimateMeasNoiseCharacteristics(time,z,1,1);
% [sZEstFriston] = estimateSmoothness(time,z);


%% Calculate higher-order derivatives of measurement noise
nZ = length(time);

zDer        = diff(z,1,2);
% zDer        = zDer - mean(zDer,2);
gausFitZDot = fitdist(zDer','Normal');

zDDer        = diff(zDer,1,2);
% zDDer        = zDDer - mean(zDDer,2);
gausFitZDDot = fitdist(zDDer','Normal');


%% Plot data
axFontSize = 30;
labelFontSize = 35;
titleFontSize = 40;

% OptiTrack data
figure('Name','Measurement noise');
box on;
% subplot(2,1,1);
plot(time,z);
xlabel('Time (s)','FontSize',labelFontSize);
ylabel('\phi (rad)','FontSize',labelFontSize);
title('Measurement noise','FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
% subplot(2,1,2);
% plot(time,normrnd(0,std(z),[1,length(time)]));


axFontSize = 15;
labelFontSize = 20;
titleFontSize = 25;
figure('Name',['Gaussian distribution of measurement noise and '...
               'derivatives']);
box on;
% xLim = [-4e-4,4e-4];
subplot(3,1,1);
histfit(z,50,'normal');
% xlim(xLim);
legend('Histogram of measurement noise','Gaussian fit');
xlabel('Noise value (rad)','FontSize',labelFontSize);
ylabel('# occurences','FontSize',labelFontSize);
title('z','FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(3,1,2);
% histfit(zDer,57,'normal');
histfit(zDer,50,'normal');
% xlim(xLim);
legend('Histogram of 1st-order derivative','Gaussian fit');
xlabel('1st-order derivative noise value (rad/s)',...
       'FontSize',labelFontSize);
ylabel('# occurences','FontSize',labelFontSize);
title('1st-order derivative of z',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
subplot(3,1,3);
% histfit(zDDer,88,'normal');
histfit(zDDer,50,'normal');
% xlim(xLim);
legend('Histogram of 2nd-order derivative','Gaussian fit');
xlabel('2nd-order derivative noise value (rad/s^2)',...
       'FontSize',labelFontSize);
ylabel('# occurences','FontSize',labelFontSize);
title('2nd-order derivative of z',...
      'FontSize',titleFontSize);
ax = gca;
ax.FontSize = axFontSize;
% 
% 
% %% Autocorrelation test
% white = normrnd(0,std(z),[1,length(time)]);
% ts = 1/120;
% n = length(time);
% tau = linspace(-time(end),time(end),2*n-1);
% s = 0.01;
% h = sqrt(1/ts*s*sqrt(pi))*exp(-tau.^2/(2*s^2));
% col = conv(h,white,'valid');
% figure;
% autocorr(white);
% figure;
% autocorr(h);
% figure;
% autocorr(col);
% 
% 
% %% Save expData data
% measNoiseData.time      = time;
% measNoiseData.z         = z;
% measNoiseData.nSamples  = 1000;
% measNoiseData.dataName  = 'ardrone2DroneSensorNoise';
% measNoiseData.SigmaEst1 = SigmaZEst1;
% measNoiseData.sEst1     = sZEstGaussian;
% measNoiseData.sEst2     = sZEstFriston;
% filename = sprintf('measNoiseData_%s',datestr(now,'dd-mm-yyyy_HH-MM'));
% save(filename,'measNoiseData');
