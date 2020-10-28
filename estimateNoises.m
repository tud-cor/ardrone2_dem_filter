%% Initialization
clear;
close all;
clc;


%% Load data
load ardrone2FlightData5_UZeroed_estNoises.mat;


%% Calculate process and measurement noise
wt = D*xt - At*xt - Bt*ut;
zt = yt - Ct*xt;

Dxt = D*xt;
AtxtBtut = At*xt + Bt*ut;


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


figure('Name','Generalized next state');
subplot(3,1,1);
plot(t,Dxt(1,:));
hold on;
plot(t,AtxtBtut(1,:));
subplot(3,1,2);
plot(t,Dxt(2,:));
hold on;
plot(t,AtxtBtut(2,:));
subplot(3,1,3);
plot(t,Dxt(3,:));
hold on;
plot(t,AtxtBtut(3,:));


figure('Name','State');
subplot(3,1,1);
plot(t,xt(1,:));
subplot(3,1,2);
plot(t,xt(2,:));
subplot(3,1,3);
plot(t,xt(3,:));

figure('Name','State der 1');
subplot(3,1,1);
plot(t,xt(4,:));
subplot(3,1,2);
plot(t,xt(5,:));
subplot(3,1,3);
plot(t,xt(6,:));

figure('Name','State der 2');
subplot(3,1,1);
plot(t,xt(7,:));
subplot(3,1,2);
plot(t,xt(8,:));
subplot(3,1,3);
plot(t,xt(9,:));

figure('Name','State der 3');
subplot(3,1,1);
plot(t,xt(10,:));
subplot(3,1,2);
plot(t,xt(11,:));
subplot(3,1,3);
plot(t,xt(12,:));
