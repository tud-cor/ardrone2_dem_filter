%% Initialization
clear;
close all;
clc;


%% Set average PWM at equilibrium
% Parrot battery
pwmEqBatP = 169.5916;

% Akku-King battery
pwmEqBatA = 171.4937;


%% Set constant coefficients
g = 9.81;
cM1 = 3.7;
cM2 = 130.9;
cA1 = cM1/2.55;
cA2 = cM2;

% Parrot battery mass
mBatP = 0.481;

% Akku-King battery mass
mBatA = 0.497;


%% Determine coefficients of own work
% Set average omegaR-thrust coefficients
cTO = [8.6e-6;
       -3.2e-4;
       0];

% Derive average PWM-thrust coefficients
cTPO = [cTO(1)*cA1^2;
        2*cTO(1)*cA1*cA2 + cTO(2)*cA1;
        cTO(1)*cA2^2 + cTO(2)*cA2 + cTO(3)];


%% Determine coefficients of Eindhoven thesis
% Given coefficients
cM  = [3.7503, 132.7387;
       3.7123, 131.5018;
       3.6891, 130.7137;
       3.7380, 132.2209];
cME = mean(cM);

% Set average PWM-thrust coefficients
cTP  = [1.5618e-4, 1.0395e-2, 0.13894;
        1.8150e-4, 8.7242e-3, 0.14425;
        1.3478e-4, 7.3295e-3, 0.11698;
        1.4306e-4, 5.7609e-3, 0.13362];
cTPE = mean(cTP)';

% Derive average omegaR-thrust coefficients
cT1Avg = cTPE(1)/cME(1)^2;
cT2Avg = -2*cME(2)*cTPE(1)/cME(1)^2 + cTPE(2)/cME(1);
cT3Avg = cTPE(1)*cME(2)^2/cME(1)^2 - ...
         cME(2)*cTPE(2)/cME(1) + cTPE(3);
cTE    = [cT1Avg;cT2Avg;cT3Avg];

% Take care of PWM range [0-255], instead of [0-100]
cTPE = cTPE./[2.55^2;2.55;1];


%% Determine coefficients of Delft thesis
% Set average omegaR-thrust coefficients
cTD = [8.386e-6;
       -3.723e-5;
       -0.03818];

% Derive average PWM-thrust coefficients (assuming same cAs as in own work)
cTPD = [cTD(1)*cA1^2;
        2*cTD(1)*cA1*cA2 + cTD(2)*cA1;
        cTD(1)*cA2^2 + cTD(2)*cA2 + cTD(3)];


%% Estimate coefficients using ground and hovering constraint
% Solve for thrust coefficients for 1 rotor (assuming rotors are the same)
% Parrot battery
cP = zeros(2,2);
cP(1,1) = cA2^2;
cP(1,2) = cA2;
cP(2,1) = cA1^2*pwmEqBatP^2 + 2*cA1*cA2*pwmEqBatP + cA2^2;
cP(2,2) = cA1*pwmEqBatP + cA2;
cTP = cP\[0;mBatP*g/4];

% Akku-King battery
cA = zeros(2,2);
cA(1,1) = cA2^2;
cA(1,2) = cA2;
cA(2,1) = cA1^2*pwmEqBatA^2 + 2*cA1*cA2*pwmEqBatA + cA2^2;
cA(2,2) = cA1*pwmEqBatA + cA2;
cTA = cA\[0;mBatA*g/4];

% Derive average omegaR-thrust coefficients
cTEs = [mean([cTP,cTA],2);0];
cT1 = cTEs(1);
cT2 = cTEs(2);

% Derive average PWM-thrust coefficients
cTPEs = [cT1*cA1^2;
         2*cT1*cA1*cA2 + cT2*cA1;
         cT1*cA2^2 + cT2*cA2 + cTEs(3)];


%% Estimate coefficients using 2 hovering constraints
cH(1,1) = cA1^2*pwmEqBatP^2 + 2*cA1*cA2*pwmEqBatP + cA2^2;
cH(1,2) = cA1*pwmEqBatP + cA2;
cH(2,1) = cA1^2*pwmEqBatA^2 + 2*cA1*cA2*pwmEqBatA + cA2^2;
cH(2,2) = cA1*pwmEqBatA + cA2;
cTH = cH\[mBatP*g/4;mBatA*g/4];

% Derive average omegaR-thrust coefficients
cTEs2 = [cTH;0];
cT1 = cTEs2(1);
cT2 = cTEs2(2);

% Derive average PWM-thrust coefficients
cTPEs2 = [cT1*cA1^2;
          2*cT1*cA1*cA2 + cT2*cA1;
          cT1*cA2^2 + cT2*cA2 + cTEs2(3)];


%% Adjustable coefficients for tests
cTTest = [1.28e-05;
          -1.68e-3;
          0];

cTPTest = [cTTest(1)*cA1^2;
           2*cTTest(1)*cA1*cA2 + cTTest(2)*cA1;
           cTTest(1)*cA2^2 + cTTest(2)*cA2 + cTTest(3)];


%% Create coefficients vectors
%  [Own work, Eindhoven thesis, Delft thesis]
cT  = [cTO,cTE,cTD,cTEs,cTEs2,cTTest];
cTP = [cTPO,cTPE,cTPD,cTPEs,cTPEs2,cTPTest];


%% Plot omegaR-thrust curve of different sources
% Set omegaR range
omegaR = linspace(0,500)';

% Determine nonlinear omegaR-thrust relation
T = [omegaR.^2,omegaR,ones(length(omegaR),1)]*cT;

% Linearize omegaR-thrust relation around batP and batA hovering equilibria
omegaRLinMin = 235;
omegaRLinMax = 500;
[~,startSample] = min(abs(omegaR-omegaRLinMin));
[~,endSample] = min(abs(omegaR-omegaRLinMax));
omegaRLin = omegaR(startSample:endSample);

omegaRHoverBatP = cA1*pwmEqBatP+cA2;
TLinBatP = omegaRLin*[2*omegaRHoverBatP,1]*cT(1:2,:) + ...
         [-omegaRHoverBatP^2,1]*cT([1,3],:);

omegaRHoverBatA = cA1*pwmEqBatA+cA2;
TLinBatA = omegaRLin*[2*omegaRHoverBatA,1]*cT(1:2,:) + ...
         [-omegaRHoverBatA^2,1]*cT([1,3],:);

% Plot
cP = [0,      0.4470, 0.7410;
      0.8500, 0.3250, 0.0980;
      0.9290, 0.6940, 0.1250;
      0.4940, 0.1840, 0.5560;
      0.4660, 0.6740, 0.1880;
      0.3010, 0.7450, 0.9330;
      0.6350, 0.0780, 0.1840];

figure('Name','Nonlinear and linear thrust curves');
hold on;
for i = 1:size(T,2)
    plot(omegaR,T(:,i),'-o','Color',cP(i,:));
    plot(omegaRLin,TLinBatP(:,i),'--','Color',cP(i,:));
    plot(omegaRLin,TLinBatA(:,i),'.-','Color',cP(i,:));
end
legend('Own work','Own work lin batP','Own work lin batP',...
       'Eindhoven','Eindhovenlin batP','Eindhoven lin batA',...
       'Delft','Delft lin batP','Delft lin batA',...
       'Estimated','Estimated lin batP','Estimated lin batA',...
       'Estimated 2','Estimated 2 lin batP','Estimated 2 lin batA',...
       'Quick test','Quick test lin batP','Quick test lin batA',...
       'location','northwest');
xlabel('\omega_r (rad/s)');
ylabel('Thrust (N)');
title('\omega_r-thrust relation');


%% Plot PWM-thrust curve of different sources
% Create PWM range
% PWM 1 from AR.Drone 2.0 is greater than PWM 0.2 from MATLAB toolbox
pwmMin = 1;
omegaRMin = cA1*pwmMin + cA2;
[~,startIdx] = min(abs(omegaR - omegaRMin));
if omegaR(startIdx) < omegaRMin
    startIdx = startIdx + 1;
end
pwm = (omegaR(startIdx:end) - cA2)/cA1;

% Create PWM-thrust data
pT = [pwm.^2,pwm,ones(length(pwm),1)]*cTP;

% Create linearized PWM polynomial data
pwmLinMin = (omegaRLinMin - cA2)/cA1;
pwmLinMax = (omegaRLinMax - cA2)/cA1;
[~,startSample] = min(abs(pwm-pwmLinMin));
[~,endSample] = min(abs(pwm-pwmLinMax));
pwmLin = pwm(startSample:endSample);

pTLinBatP = pwmLin*[2*pwmEqBatP,1]*cTP(1:2,:) + ...
         [-pwmEqBatP^2,1]*cTP([1,3],:);

pTLinBatA = pwmLin*[2*pwmEqBatA,1]*cTP(1:2,:) + ...
         [-pwmEqBatA^2,1]*cTP([1,3],:);

% Create PWM-thrust and linearization data for Eindhoven thesis
% pwmE = pwm/2.55;
% pwmLinE = pwmLin/2.55;
% pTE = [pwmE.^2,pwmE,ones(length(pwmE),1)]*cTPE;
% 
% pTLinBatPE = pwmLinE*[2*pwmEqBatP/2.55,1]*cTPE(1:2) + ...
%          [-(pwmEqBatP/2.55)^2,1]*cTPE([1,3]);
% 
% pTLinBatAE = pwmLinE*[2*pwmEqBatA/2.55,1]*cTPE(1:2) + ...
%          [-(pwmEqBatA/2.55)^2,1]*cTPE([1,3]);

% Plot
figure('Name','Nonlinear and linear thrust curves per PWM value');
hold on;
for i = 1:size(pT,2)
    plot(pwm,pT(:,i),'-o','Color',cP(i,:));
    plot(pwmLin,pTLinBatP(:,i),'--','Color',cP(i,:));
    plot(pwmLin,pTLinBatA(:,i),'.-','Color',cP(i,:));
end
xline(pwmEqBatP);
yline(mBatP*g/4);
xline(pwmEqBatA);
yline(mBatA*g/4);
legend('Own work','Own work lin batP','Own work lin batP',...
       'Eindhoven','Eindhoven lin batP','Eindhoven lin batA',...
       'Delft','Delft lin batP','Delft lin batA',...
       'Estimated','Estimated lin batP','Estimated lin batA',...
       'Estimated 2','Estimated 2 lin batP','Estimated 2 lin batA',...
       'Quick test','Quick test lin batP','Quick test lin batA',...
       'location','northwest');
xlabel('AR.Drone 2.0 PWM (-)');
ylabel('Thrust (N)');
title('PWM-thrust relation');


%% Shift up identified omegaR-thrust curve
