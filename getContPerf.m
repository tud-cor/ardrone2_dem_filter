%% Initialization
clear;
close all;
clc;


% %% Get tum_ardrone controller simulation results -- splitted into
% % different time slots to prevent memory problems when reading out
% % recorded messages
% % Retrieve bag file
% cd ~/.ros;
% bag = rosbag("tum_ardrone_controller.bag");
% cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;
% 
% % Select topics that need to be stored
% topics.modelInput = 1;
% topics.modelStates = 1;
% 
% % Set time interval with respect to start of rosbag recording
% time = [0,20];
% 
% % Get data
% topicsOut = storeBagdata(bag, topics, time);
% 
% if topics.modelInput
%     modelInputTime = topicsOut.modelInput.time;
%     force = topicsOut.modelInput.force;
%     torque = topicsOut.modelInput.torque;
% end
% 
% if topics.modelStates
%     modelStatesTime = topicsOut.modelStates.time;
%     pos = topicsOut.modelStates.pos;
% end
% 
% % Save data to mat file
% save('tum_0-20s.mat','modelInputTime','force','torque', ...
%      'modelStatesTime','pos');
% clear;
% 
% % Set time interval with respect to start of rosbag recording
% time = [20,40];
% 
% % Get data
% topicsOut = storeBagdata(bag, topics, time);
% 
% if topics.modelInput
%     modelInputTime = topicsOut.modelInput.time;
%     force = topicsOut.modelInput.force;
%     torque = topicsOut.modelInput.torque;
% end
% 
% if topics.modelStates
%     modelStatesTime = topicsOut.modelStates.time;
%     pos = topicsOut.modelStates.pos;
% end
% 
% % Save data to mat file
% save('tum_20-40s.mat','modelInputTime','force','torque', ...
%      'modelStatesTime','pos');
% clear;
% 
% % Set time interval with respect to start of rosbag recording
% time = [40,70];
% 
% % Get data
% topicsOut = storeBagdata(bag, topics, time);
% 
% if topics.modelInput
%     modelInputTime = topicsOut.modelInput.time;
%     force = topicsOut.modelInput.force;
%     torque = topicsOut.modelInput.torque;
% end
% 
% if topics.modelStates
%     modelStatesTime = topicsOut.modelStates.time;
%     pos = topicsOut.modelStates.pos;
% end
% 
% % Save data to mat file
% save('tum_40-70s.mat','modelInputTime','force','torque', ...
%      'modelStatesTime','pos');
% clear;
% 
% 
% %% Get tum_ardrone controller simulation results
% % Retrieve bag file
% cd ~/.ros;
% bag = rosbag("rotors_simulator_controller.bag");
% cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;
% 
% % Select topics that need to be stored
% topics.modelStates = 1;
% topics.ardroneMotorSpeed = 1;
% 
% % Set time interval with respect to start of rosbag recording
% time = [0,70];
% 
% % Get data
% topicsOut = storeBagdata(bag, topics, time);
% 
% if topics.modelStates
%     modelStatesTime = topicsOut.modelStates.time;
%     pos = topicsOut.modelStates.pos;
% end
% 
% if topics.ardroneMotorSpeed
%     rotorTime = topicsOut.motorSpeed.time;
%     rotorAngVel = topicsOut.motorSpeed.angVel;
% end
% 
% % Save data to mat file
% save('rotors_0-70s.mat', 'modelStatesTime','pos','rotorTime',...
%      'rotorAngVel');
% clear;
% 
% 
% %% Construct data tum_ardrone controller
% load tum_0-20s.mat;
% tumModelTime = modelStatesTime;
% tumPos = pos;
% tumInputTime = modelInputTime;
% tumForce = force;
% tumTorque = torque;
% 
% load tum_20-40s.mat;
% tumModelTime = [tumModelTime,modelStatesTime];
% tumPos = [tumPos,pos];
% tumInputTime = [tumInputTime,modelInputTime];
% tumForce = [tumForce,force];
% tumTorque = [tumTorque,torque];
% 
% load tum_40-70s.mat;
% tumModelTime = [tumModelTime,modelStatesTime];
% tumPos = [tumPos,pos];
% tumInputTime = [tumInputTime,modelInputTime];
% tumForce = [tumForce,force];
% tumTorque = [tumTorque,torque];
% 
% 
% %% Construct data rotors_simulator controller
% load rotors_0-70s.mat;
% rotorsModelTime = modelStatesTime;
% rotorsPos = pos;
% rotorsInputTime = rotorTime;
% rotorsAngVel = rotorAngVel;
% 
% 
% %% Construct trajectory
% n = 53;
% t = linspace(0,50,n)';
% f = 1/50;
% omega = 2*pi*f;
% a = 0.5;
% 
% trajPos = zeros(3,n);
% trajPos(1,:) = rmvFPInacc(a*sin(omega*t));
% trajPos(2,:) = rmvFPInacc(sin(omega*t).*cos(omega*t));
% trajPos(3,:) = rmvFPInacc(trajPos(1,:)+1+max(trajPos(1,:)));
% 
% 
% %% Save simulation data to mat file
% save('controllerComparison.mat','tumModelTime','tumPos',...
%      'tumInputTime','tumForce','tumTorque','rotorsModelTime',...
%      'rotorsPos','rotorsInputTime','rotorsAngVel','trajPos');
% 
% 
%% Load simulation data
load controllerComparison.mat;


%% Select trajectory data from simulations
for modelIdx1 = 1:length(tumModelTime)
    if tumModelTime(modelIdx1) >= 17.3
        break;
    end
end
for modelIdx2 = modelIdx1:length(tumModelTime)
    if tumModelTime(modelIdx2) > 62.3
        break;
    end
end
modelIdx2 = modelIdx2 - 1;
tumModelTimeTraj = tumModelTime(modelIdx1:modelIdx2);
tumPosTraj = tumPos(:,modelIdx1:modelIdx2);

for inputIdx1 = 1:length(tumInputTime)
    if tumInputTime(inputIdx1) >= 17.3
        break;
    end
end
for inputIdx2 = inputIdx1:length(tumInputTime)
    if tumInputTime(inputIdx2) > 62.3
        break;
    end
end
inputIdx2 = inputIdx2 - 1;
tumInputTimeTraj = tumInputTime(inputIdx1:inputIdx2);
tumForceTraj = tumForce(:,inputIdx1:inputIdx2);
tumTorqueTraj = tumTorque(:,inputIdx1:inputIdx2);


for modelIdx1 = 1:length(rotorsModelTime)
    if rotorsModelTime(modelIdx1) >= 10.5
        break;
    end
end
for modelIdx2 = modelIdx1:length(rotorsModelTime)
    if rotorsModelTime(modelIdx2) > 17
        break;
    end
end
modelIdx2 = modelIdx2 - 1;
rotorsModelTimeTraj = rotorsModelTime(modelIdx1:modelIdx2);
rotorsPosTraj = rotorsPos(:,modelIdx1:modelIdx2);

for inputIdx1 = 1:length(rotorsInputTime)
    if rotorsInputTime(inputIdx1) >= 10.5
        break;
    end
end
for inputIdx2 = inputIdx1:length(rotorsInputTime)
    if rotorsInputTime(inputIdx2) > 17
        break;
    end
end
inputIdx2 = inputIdx2 - 1;
rotorsInputTimeTraj = rotorsInputTime(inputIdx1:inputIdx2);
rotorsAngVelTraj = rotorsAngVel(:,inputIdx1:inputIdx2);


%% Plot data
figure('Name','Quadrotor trajectory following');
plot3(tumPosTraj(1,:),tumPosTraj(2,:),tumPosTraj(3,:));
hold on;
plot3(rotorsPosTraj(1,:),rotorsPosTraj(2,:),rotorsPosTraj(3,:));
plot3(trajPos(1,:),trajPos(2,:),trajPos(3,:));
scatter3(trajPos(1,:),trajPos(2,:),trajPos(3,:),'o',...
         'MarkerEdgeColor',[0.9290, 0.6940, 0.1250],...
         'MarkerFaceColor',[0.9290, 0.6940, 0.1250]);
xlim([-0.6,0.6]);
ylim([-0.6,0.6]);
zlim([0,2]);
legend('tum\_ardrone','rotors\_simulator','Desired trajectory');
ax = gca;
ax.FontSize = 40;
title('Quadrotor trajectory following','FontSize',60);
xlabel('x (m)','FontSize',40);
ylabel('y (m)','FontSize',40);
zlabel('z (m)','FontSize',40);
% 
% 
% figure('Name','tum_ardrone controller position');
% plot(tumModelTime,tumPos(1,:));
% hold on;
% plot(tumModelTime,tumPos(2,:));
% plot(tumModelTime,tumPos(3,:));
% xline(17.3,'Color',[0.4940, 0.1840, 0.5560],'LineWidth',3);
% xline(62.3,'Color',[0.4940, 0.1840, 0.5560],'LineWidth',3);
% xlim([0,70]);
% legend('x','y','z');
% ax = gca;
% ax.FontSize = 40;
% title('tum\_ardrone controller','FontSize',60);
% xlabel('t (s)','FontSize',40);
% ylabel('pos (m)','FontSize',40);
% 
% figure('Name','tum_ardrone controller forces');
% sgtitle('Forces on quadrotor during trajectory','FontSize',60);
% subplot(3,2,1);
% plot(tumInputTimeTraj,tumForceTraj(1,:));
% xlabel('t (s)');
% ylabel('f (N)');
% ax = gca;
% ax.FontSize = 20;
% subplot(3,2,2);
% plot(tumInputTimeTraj,tumForceTraj(1,:));
% xlabel('t (s)');
% ylabel('f (N)');
% ax = gca;
% ax.FontSize = 20;
% subplot(3,2,3);
% plot(tumInputTimeTraj,tumForceTraj(2,:));
% xlabel('t (s)');
% ylabel('f (N)');
% ax = gca;
% ax.FontSize = 20;
% subplot(3,2,4);
% plot(tumInputTimeTraj,tumForceTraj(2,:));
% xlabel('t (s)');
% ylabel('f (N)');
% ax = gca;
% ax.FontSize = 20;
% subplot(3,2,5);
% plot(tumInputTimeTraj,tumForceTraj(3,:));
% xlabel('t (s)');
% ylabel('f (N)');
% ax = gca;
% ax.FontSize = 20;
% ylim([-5,35]);
% subplot(3,2,6);
% plot(tumInputTimeTraj,tumForceTraj(3,:));
% xlabel('t (s)');
% ylabel('f (N)');
% ax = gca;
% ax.FontSize = 20;
% ylim([-5,35]);
% 
% figure('Name','tum_ardrone controller torques');
% sgtitle('Torques on quadrotor during trajectory','FontSize',60);
% subplot(3,2,1);
% plot(tumInputTimeTraj,tumTorqueTraj(1,:));
% xlabel('t (s)');
% ylabel('T (Nm)');
% ax = gca;
% ax.FontSize = 20;
% subplot(3,2,2);
% plot(tumInputTimeTraj,tumTorqueTraj(1,:));
% xlabel('t (s)');
% ylabel('T (Nm)');
% ax = gca;
% ax.FontSize = 20;
% subplot(3,2,3);
% plot(tumInputTimeTraj,tumTorqueTraj(2,:));
% xlabel('t (s)');
% ylabel('T (Nm)');
% ax = gca;
% ax.FontSize = 20;
% subplot(3,2,4);
% plot(tumInputTimeTraj,tumTorqueTraj(2,:));
% xlabel('t (s)');
% ylabel('T (Nm)');
% ax = gca;
% ax.FontSize = 20;
% subplot(3,2,5);
% plot(tumInputTimeTraj,tumTorqueTraj(3,:));
% xlabel('t (s)');
% ylabel('T (Nm)');
% ax = gca;
% ax.FontSize = 20;
% subplot(3,2,6);
% plot(tumInputTimeTraj,tumTorqueTraj(3,:));
% xlabel('t (s)');
% ylabel('T (Nm)');
% ax = gca;
% ax.FontSize = 20;
% 
% 
% figure('Name','rotors_simulator controller position');
% plot(rotorsModelTime,rotorsPos(1,:));
% hold on;
% plot(rotorsModelTime,rotorsPos(2,:));
% plot(rotorsModelTime,rotorsPos(3,:));
% xline(10.5,'Color',[0.4940, 0.1840, 0.5560],'LineWidth',3);
% xline(17,'Color',[0.4940, 0.1840, 0.5560],'LineWidth',3);
% xlim([0,70]);
% legend('x','y','z');
% ax = gca;
% ax.FontSize = 40;
% title('rotors\_simulator controller','FontSize',60);
% xlabel('t (s)','FontSize',40);
% ylabel('pos (m)','FontSize',40);
% 
% figure('Name','rotors_simulator controller model input');
% subplot(1,2,1);
% plot(rotorsInputTime,rotorsAngVel(1,:));
% hold on;
% plot(rotorsInputTime,rotorsAngVel(2,:));
% plot(rotorsInputTime,rotorsAngVel(3,:));
% plot(rotorsInputTime,rotorsAngVel(4,:));
% ylim([-1000,1000]);
% xlim([5,35]);
% legend('\omega_1','\omega_2','\omega_3','\omega_4', 'Location', 'best');
% ax = gca;
% ax.FontSize = 30;
% title('Rotor angular velocities','FontSize',40);
% xlabel('t (s)');
% ylabel('\omega (rad/s)');
% subplot(1,2,2);
% plot(rotorsInputTimeTraj,rotorsAngVelTraj(1,:));
% hold on;
% plot(rotorsInputTimeTraj,rotorsAngVelTraj(2,:));
% plot(rotorsInputTimeTraj,rotorsAngVelTraj(3,:));
% plot(rotorsInputTimeTraj,rotorsAngVelTraj(4,:));
% ylim([-1000,1000]);
% legend('\omega_1','\omega_2','\omega_3','\omega_4', 'Location', 'best');
% ax = gca;
% ax.FontSize = 30;
% title('Rotor angular velocities during trajectory','FontSize',40);
% xlabel('t (s)');
% ylabel('\omega (rad/s)');


%% Function definitions
% function output = rmvFPInacc(input)
% thres = 1e-10;
% 
% output = input;
% for i = 1:length(output)
%     if (abs(output(i)) < thres)
%         output(i) = 0;
%     end
% end
% end