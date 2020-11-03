%% Initialization
clear;
close all;
clc;


%% Set average PWM at equilibrium
% Parrot battery
pwmEqBatP = 169.5916;

% Akku-King battery
pwmEqBatA = 171.4937;


%% Set coefficients
g = 9.81;
cM1 = 3.7;
cM2 = 130.9;
cA1 = cM1/2.55;
cA2 = cM2;

% Parrot battery
mBatP = 0.481;

% Akku-King battery
mBatA = 0.497;


%% Solve for thrust coefficients
% Parrot battery
c1BatP = 4*cA1^2*pwmEqBatP^2 + 8*cA1*cA2*pwmEqBatP + 4*cA2^2;
c2BatP = 4*cA1*pwmEqBatP + 4*cA2;

cT1BatP = mBatP*g/c1BatP*(1/(1-c2BatP/c1BatP*cA2));
cT2BatP = -cA2*cT1BatP;

% Akku-King battery
c1BatA = 4*cA1^2*pwmEqBatA^2 + 8*cA1*cA2*pwmEqBatA + 4*cA2^2;
c2BatA = 4*cA1*pwmEqBatA + 4*cA2;

cT1BatA = mBatA*g/c1BatA*(1/(1-c2BatA/c1BatA*cA2));
cT2BatA = -cA2*cT1BatA;


%% Determine average thrust coefficients
cT1 = mean([cT1BatP,cT1BatA]);
cT2 = mean([cT2BatP,cT2BatA]);


%% Compare with identified coefficients in different theses
load thrust_torque_comp_theses.mat;
omegaR = linspace(0,500)';

% Thrust
Tq = omegaR.^2*thrustPoly.p1;
T2wc = [omegaR.^2,omegaR]*thrustPoly.p2;
T2 = [omegaR.^2,omegaR,ones(length(omegaR),1)]*thrustPoly.p3;
TEst = [omegaR.^2,omegaR]*[cT1;cT2];

pwmEq = mean([pwmEqBatP,pwmEqBatA]);
minPwm = 235;
maxPwm = 500;
[~,startSample] = min(abs(omegaR-minPwm));
[~,endSample] = min(abs(omegaR-maxPwm));
omegaRLin = omegaR(startSample:endSample);
omegaRHoverBatP = cA1*pwmEqBatP+cA2;
T2LinBatP = omegaRLin*[2*omegaRHoverBatP,1]*...
        [thrustPoly.p3(1:2,:),[cT1;cT2]] + ...
         [-omegaRHoverBatP^2,1]*[thrustPoly.p3([1,3],:),[cT1;0]];
omegaRHoverBatA = cA1*pwmEqBatA+cA2;
T2LinBatA = omegaRLin*[2*omegaRHoverBatA,1]*...
        [thrustPoly.p3(1:2,:),[cT1;cT2]] + ...
         [-omegaRHoverBatA^2,1]*[thrustPoly.p3([1,3],:),[cT1;0]];

c = [0,      0.4470, 0.7410;
     0.8500, 0.3250, 0.0980;
     0.9290, 0.6940, 0.1250;
     0.4940, 0.1840, 0.5560];

figure('Name','Nonlinear and linear thrust curves');
hold on;
plot(omegaR,T2wc(:,1),'-o','Color',c(1,:));
plot(omegaR,T2(:,2),'-o','Color',c(2,:));
plot(omegaR,T2(:,3),'-o','Color',c(3,:));
plot(omegaR,TEst,'-o','Color',c(4,:));
for i = 1:4
    plot(omegaRLin,T2LinBatP(:,i),'--','Color',c(i,:));
end
for i = 1:4
    plot(omegaRLin,T2LinBatA(:,i),'.-','Color',c(i,:));
end
legend('Own work','Delft thesis','Twente thesis','Estimated',...
       'Own work lin batP','Delft thesis lin batP',...
       'Twente thesis lin batP','Estimated lin batP',...
       'Own work lin batA','Delft thesis lin batA',...
       'Twente thesis lin batA','Estimated lin batA',...
       'location','northwest');
