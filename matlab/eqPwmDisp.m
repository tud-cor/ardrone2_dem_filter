%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate equilibrium PWM values
%
% Script to read out hover experiment data per battery type
% (Parrot=batP/Akku-King=batA) to obtain the average PWM value per rotor as
% well as for all rotors together during hovering.
% 
% Author:        Dennis Benders, TU Delft, CoR
% Last modified: 21.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear;
close all;
clc;


%% Load batP data
load hoverBatP.mat;
pwmP = [pwmAvg1,pwmAvg2,pwmAvg4,pwmAvg6,pwmAvg7,pwmAvg9,pwmAvg10];
pwmAvgP = mean(pwmP)

pwmAvgsP = mean([pwmAvgs1,pwmAvgs2,pwmAvgs4,pwmAvgs6,pwmAvgs7,pwmAvgs9,...
                 pwmAvgs10],2)

clear pwmAvg1 pwmAvg2 pwmAvg4 pwmAvg6 pwmAvg7 pwmAvg9 pwmAvg10


%% Load batA data
load hoverBatA.mat;
pwmA = [pwmAvg1,pwmAvg2,pwmAvg3,pwmAvg4,pwmAvg6,pwmAvg8,pwmAvg10];
pwmAvgA = mean(pwmA)

pwmAvgsA = mean([pwmAvgs1,pwmAvgs2,pwmAvgs3,pwmAvgs4,pwmAvgs6,pwmAvgs8,...
                 pwmAvgs10],2)
