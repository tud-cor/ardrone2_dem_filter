%% Initialization
clear;
close all;
clc;

% % Parameters
% eulThres = 6;   %minimum difference in Euler angle to compensate for jumps
% 
% 
% %% Set variables
% % Retrieve bag file
% cd ~/.ros;
% bag = rosbag("ardrone2_exp_2020-07-24_4.bag");
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
% topics.ardroneNavdata = 0;
% topics.ardroneOdom = 0;
% topics.rotorsMotorSpeed = 0;
% 
% % Set time interval with respect to start of rosbag recording
% time = [0,40];
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
% if topics.ardroneNavdata
%     ardroneNavdataStampTime = topicsOut.ardroneNavdata.stampTime;
%     ardroneNavdataRecordTime = topicsOut.ardroneNavdata.recordTime;
%     ardroneNavdataMotor = topicsOut.ardroneNavdata.motor;
%     ardroneNavdataRot = topicsOut.ardroneNavdata.rot;
%     ardroneNavdataVLin = topicsOut.ardroneNavdata.vLin;
%     ardroneNavdataALin = topicsOut.ardroneNavdata.aLin;
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
% % Plot position data to search for time where x and y are constant
% figure('Name','OptiTrack position data');
% hold on;
% plot(optitrackStampTime,optitrackPos(1,:),'-o');
% plot(optitrackStampTime,optitrackPos(2,:),'-o');
% plot(optitrackStampTime,optitrackPos(3,:),'-o');
% 
% % Select data samples to use
% prompt = {'Enter index of 1st data sample:',...
%           'Enter index of last data sample:'};
% dlgtitle = 'Data selection';
% dims = [1 35];
% definput = {'1',num2str(length(optitrackStampTime))};
% answer = inputdlg(prompt,dlgtitle,dims,definput);
% otStart = round(str2double(answer{1}));
% otEnd = round(str2double(answer{2}));
% 
% % Select proper OptiTrack data
% optitrackStampTime = optitrackStampTime(otStart:otEnd);
% optitrackPos = optitrackPos(:,otStart:otEnd);
% optitrackOrientQuat = optitrackOrientQuat(:,otStart:otEnd);
% 
% 
% %% Convert OptiTrack quaternions to ZYX Euler angles
% % Convert quaternions to Euler angles
% optitrackOrientQuat = optitrackOrientQuat';
% orient = zeros(size(optitrackOrientQuat,1),3);
% for i = 1:size(optitrackOrientQuat,1)
%     orient(i,:) = quat2eul(optitrackOrientQuat(i,:));
% end
% orient = orient';
% 
% % Remove jumps of 2*pi in angle data and ensure the angles are centered
% % around 0 rad
% optitrackOrient = unwrap(orient,eulThres,2);
% for i = 1:3
%     if mean(optitrackOrient(i,:)) > eulThres/2
%         optitrackOrient(i,:) = optitrackOrient(i,:) - pi;
%     elseif mean(optitrackOrient(i,:)) < -eulThres/2
%         optitrackOrient(i,:) = optitrackOrient(i,:) + pi;
%     end
% end
% 
% figure('Name','OptiTrack orientation data');
% subplot(3,1,1);
% plot(optitrackStampTime,optitrackOrient(1,:),'-o');
% title('\phi');
% subplot(3,1,2);
% plot(optitrackStampTime,optitrackOrient(2,:),'-o');
% title('\theta');
% subplot(3,1,3);
% plot(optitrackStampTime,optitrackOrient(3,:),'-o');
% title('\psi');
% 
% 
% %% Interpolate data
% % Sample time
% fs = 120;
% measNoiseData.sampleTime = 1/fs;
% 
% % OptiTrack data
% data.time = optitrackStampTime;
% data.value = [optitrackPos;optitrackOrient];
% tmp = interpolate(measNoiseData.sampleTime,data);
% time = tmp.time;
% z = tmp.value;
% 
% 
% %% Ensure data start at time 0 and data is zeroed
% time = time - time(1);
% z = z - z(:,1);
% figure('Name','Position');
% plot(time,z(1:3,:)');
% figure('Name','Orientation');
% plot(time,z(4:6,:)');


%% To speed up the process: code above is commented and a dedicated mat
%  file is loaded
load measNoiseTest.mat;


%% Calculate noise characteristics of OptiTrack position and orientation
%  states
% [f,pZ1] = getFFT(time,z);
% plot(f,pZ1);
% legend('x','y','z','\phi','\theta','\psi');
% ylim([0,0.3e-3]);

% for i = 1:ny
%     figure('Name',['Frequency spectrum of z' num2str(i)]);
%     plot(f,pZ1(i,:));
% end

time = time(1:1000);
z = z(:,1:1000);

[SigmaZEst1,sZEst1] = estimateNoiseCharacteristics(time,z,0,0);

[sZEst2] = estimateSmoothness(time,z);


%% Save expData data
measNoiseData.time      = time;
measNoiseData.z         = z;
measNoiseData.nSamples  = 1000;
measNoiseData.dataName  = 'exp_24-7_0-40s';
measNoiseData.SigmaEst1 = SigmaZEst1;
measNoiseData.sEst1     = sZEst1;
measNoiseData.sEst2     = sZEst2;
filename = sprintf('measNoiseData_%s',datestr(now,'dd-mm-yyyy_HH-MM'));
save(filename,'measNoiseData');
