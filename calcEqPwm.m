%% Initialization
clear;
close all;
clc;


%% Load batP data
load hoverBatP.mat;
pwmAvgP = mean([pwmAvg1,pwmAvg2,pwmAvg4,pwmAvg6,pwmAvg7,pwmAvg9,pwmAvg10])

pwmAvgsP = mean([pwmAvgs1,pwmAvgs2,pwmAvgs4,pwmAvgs6,pwmAvgs7,pwmAvgs9,...
                 pwmAvgs10],2)

clear pwmAvg1 pwmAvg2 pwmAvg4 pwmAvg6 pwmAvg7 pwmAvg9 pwmAvg10

%% Load batA data
load hoverBatA.mat;
pwmAvgA = mean([pwmAvg1,pwmAvg2,pwmAvg3,pwmAvg4,pwmAvg6,pwmAvg8,pwmAvg10])

pwmAvgsA = mean([pwmAvgs1,pwmAvgs2,pwmAvgs3,pwmAvgs4,pwmAvgs6,pwmAvgs8,...
                 pwmAvgs10],2)
