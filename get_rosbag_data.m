%% Initialization
clear all;
close all;
clc;


%% Set variables
% Retrieve bag file
cd ~/.ros;
bag = rosbag("force-torque-meas_2020-04-29-12-32-11.bag");
cd ~/ardrone2_ws/src/ardrone2_dem/dem/matlab;

topics.model_input = 1;

time = [3, 10];


%% Get data
topics_out = storeBagdata(bag, topics, time);

time = topics_out.model_input.time;
force = topics_out.model_input.force;
torque = topics_out.model_input.torque;


%% Plot data
figure('Name', 'Force plots');

subplot(3,1,1);
plot(time, force(1,:), 'LineStyle', '-', 'Marker', '.');
title('f_x');
xlabel('Time (s)');
ylabel('Force (N?)');

subplot(3,1,2);
plot(time, force(2,:), 'LineStyle', '-', 'Marker', '.');
title('f_y');
xlabel('Time (s)');
ylabel('Force (N?)');

subplot(3,1,3);
plot(time, force(3,:), 'LineStyle', '-', 'Marker', '.');
title('f_z');
xlabel('Time (s)');
ylabel('Force (N?)');


figure('Name', 'Torque plots');

subplot(3,1,1);
plot(time, torque(1,:), 'LineStyle', '-', 'Marker', '.');
title('\tau_x');
xlabel('Time (s)');
ylabel('Torque (Nm?)');

subplot(3,1,2);
plot(time, torque(2,:), 'LineStyle', '-', 'Marker', '.');
title('\tau_y');
xlabel('Time (s)');
ylabel('Torque (Nm?)');

subplot(3,1,3);
plot(time, torque(3,:), 'LineStyle', '-', 'Marker', '.');
title('\tau_z');
xlabel('Time (s)');
ylabel('Torque (Nm?)');