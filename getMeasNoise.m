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
% Measurement noise dimensions
ny = size(z,1);
n = size(z,2);

% Calculate frequency spectrum of measurement noise
for stateNr = 1:6
%     z(stateNr,:) = z(stateNr,:) - mean(z(stateNr,:));
    fs = zeros(1,n-1);
    for i = 1:n-1
        fs(i) = time(i+1) - time(i);
    end
    fs = 1/mean(fs);
    f = fs*(0:(n/2))/n;

    freqZ = fft(z(stateNr,:),n,2);
    pZ2 = abs(freqZ/n);
    pZ1 = pZ2(:,1:n/2+1);
    pZ1(:,2:end-1) = 2*pZ1(:,2:end-1);
%     figure('Name','Measurement noise in frequency domain');
%     plot(f,pZ1);
end

% Standard deviation
muZ = mean(z,2);
muZAvg = mean(muZ);
SigmaZ = diag(std(z,1,2));
sigmaZAvg = mean(diag(SigmaZ));

% Kernel width of Gaussian filter that is assumed to produce coloured noise
% from white noise
numLags = 100;
numStd = 1;
acZ = zeros(ny,numLags+1);
lagsZ = zeros(ny,numLags+1);
boundsZ = zeros(ny,2);
for i = 1:ny
    [acZ(i,:),lagsZ(i,:),boundsZ(i,:)] = ...
        autocorr(z(i,:),'NumLags',numLags,'NumSTD',1);
%     figure('Name',num2str(i));
%     stem(lagsZ(i,:),acZ(i,:),'filled');
%     hold on
%     plot(lagsZ(i,:),boundsZ(i,:)'*ones(size(lagsZ(i,:))),'r');
end
rng(1);
tau = linspace(-time(end),time(end),2*n-1);
muOmegaRef = muZAvg;
SigmaOmegaRef = sigmaZAvg;
omegaRef = normrnd(muOmegaRef,SigmaOmegaRef,[ny,n]);
mseResult.sRef = 0.001:0.01:1;
mseResult.mseRef = zeros(ny,length(mseResult.sRef));
for i = 1:length(mseResult.sRef)
    s = mseResult.sRef(i);
    h = sqrt(1/fs*s*sqrt(pi))*exp(-tau.^2/(2*s^2));
    ref = zeros(ny,n);
    acRef = zeros(ny,numLags+1);
    lagsRef = zeros(ny,numLags+1);
    boundsRef = zeros(ny,2);
    for j = 1:ny
        ref(j,:) = conv(h,omegaRef(j,:),'valid');
        [acRef(j,:),lagsRef(j,:),boundsRef(j,:)] = ...
            autocorr(ref(j,:),'NumLags',numLags,'NumSTD',numStd);
        mseResult.mseRef(j,i) = mean((acZ(j,:)-acRef(j,:)).^2);
    end
end

for i = 1:ny
    figure('Name',strcat(['Estimated kernel width for z' num2str(i)],...
                         ' using LS'));
    plot(mseResult.sRef,mseResult.mseRef(i,:),'-o');
    hold on;
    [~,idx] = min(mseResult.mseRef(i,:));
    xline(mseResult.sRef(idx),'Color',[0 0.4470 0.7410]);
    legend(['MSE for sZ ' num2str(mseResult.sRef(i)) '-' ...
            num2str(mseResult.sRef(end))],'sZ');
end

%% Save expData data
% measNoiseData.time = time;
% measNoiseData.z = z;
% measNoiseData.dataName = 'exp_24-7_0-40s';
% filename = sprintf('bagdata_%s', datestr(now,'dd-mm-yyyy_HH-MM'));
% save(filename, 'measNoiseData');
