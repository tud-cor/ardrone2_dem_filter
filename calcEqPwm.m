%% Initialization
clear;
close all;
clc;


%% Load batP data
load hoverBatP.mat;
pwmAvgB = mean([pwmAvg1,pwmAvg2,pwmAvg4,pwmAvg6,pwmAvg7,pwmAvg9,pwmAvg10])
clear pwmAvg1 pwmAvg2 pwmAvg4 pwmAvg6 pwmAvg7 pwmAvg9 pwmAvg10

% TODO Show averages for every motor-rotor combination

%% Load batA data
load hoverBatA.mat;
pwmAvgA = mean([pwmAvg1,pwmAvg2,pwmAvg3,pwmAvg4,pwmAvg6,pwmAvg8,pwmAvg10])

% TODO Show averages for every motor-rotor combination