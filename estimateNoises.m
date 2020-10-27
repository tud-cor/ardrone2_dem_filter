%% Initialization
clear;
close all;
clc;


%% Load data
load ardrone2FlightData5_UZeroed_estNoises.mat;


%% Calculate process and measurement noise
wt = D*xt - At*xt - Bt*ut;
zt = yt - Ct*xt;


%% Determine noise characteristics of process and measurement noise
figure('Name','Process noise');
subplot(3,1,1);
plot(t,wt(1,:));
subplot(3,1,2);
plot(t,wt(2,:));
subplot(3,1,3);
plot(t,wt(3,:));

figure('Name','Measurement noise');
subplot(3,1,1);
plot(t,zt(1,:));
subplot(3,1,2);
plot(t,zt(2,:));
subplot(3,1,3);
plot(t,zt(3,:));
