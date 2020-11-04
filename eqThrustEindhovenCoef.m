%% Initialization
clear;
close all;
clc;


%% Given coefficients
cM = [3.7503, 132.7387;
      3.7123, 131.5018;
      3.6891, 130.7137;
      3.7380, 132.2209]';
cMAvg = mean(cM,2);

cTP = [1.5618e-4, 1.0395e-2, 0.13894;
       1.8150e-4, 8.7242e-3, 0.14425;
       1.3478e-4, 7.3295e-3, 0.11698;
       1.4306e-4, 5.7609e-3, 0.13362]';
cTPAvg = mean(cTP,2);


%% Derive cT1, cT2 and cT3 for each motor-rotor combination:
%  [motor 1, motor2, motor 3, motor 4]
cT = zeros(3,4);
for i = 1:4
    cT(1,i) = cTP(1,i)/cM(1,i)^2;
    cT(2,i) = -2*cM(2,i)*cTP(1,i)/cM(1,i)^2 + cTP(2,i)/cM(1,i);
    cT(3,i) = cTP(1,i)*cM(2,i)^2/cM(1,i)^2 - cM(2,i)*cTP(2,i)/cM(1,i) + ...
              cTP(3,i);
end


%% Derive average cT1, cT2 and cT3 from given coefficients
cT1Avg = cTPAvg(1)/cMAvg(1)^2;
cT2Avg = -2*cMAvg(2)*cTPAvg(1)/cMAvg(1)^2 + cTPAvg(2)/cMAvg(1);
cT3Avg = cTPAvg(1)*cMAvg(2)^2/cMAvg(1)^2 - ...
         cMAvg(2)*cTPAvg(2)/cMAvg(1) + cTPAvg(3);
cTAvg = [cT1Avg;cT2Avg;cT3Avg];


%% Plot omegaR-thrust relation
omegaR = linspace(0,500)';
T = [omegaR.^2,omegaR,ones(length(omegaR),1)]*cT;
TAvg = [omegaR.^2,omegaR,ones(length(omegaR),1)]*cTAvg;

c = [0,      0.4470, 0.7410;
     0.8500, 0.3250, 0.0980;
     0.9290, 0.6940, 0.1250;
     0.4940, 0.1840, 0.5560;
     0.4660, 0.6740, 0.1880;
     0.3010, 0.7450, 0.9330;
     0.6350, 0.0780, 0.1840];

figure('Name','Eindhoven omegaR-thrust curve');
hold on;
for i = 1:4
    plot(omegaR,T(:,i),'Color',c(i,:));
end
plot(omegaR,TAvg,'Color',c(5,:));
legend('Motor 1','Motor 2','Motor 3','Motor 4','Motor avg');
xlabel('\omega_r (rad/s)');
ylabel('Thrust (N)');
title('Eindhoven \omega_r-thrust curve');


%% Create PWM characteristic coefficients from omegaR-thrust relation
cTP2 = [cTAvg(1)*cMAvg(1)^2;
        2*cTAvg(1)*cMAvg(1)*cMAvg(2) + cTAvg(2)*cMAvg(1);
        cTAvg(1)*cMAvg(2)^2 + cTAvg(2)*cMAvg(2)];


%% Plot PWM-thrust relation
pwm = (omegaR - cMAvg(2))/cMAvg(1);
TP = [pwm.^2,pwm,ones(length(pwm),1)]*cTP;
TPAvg = [pwm.^2,pwm,ones(length(pwm),1)]*cTPAvg;
TP2 = [pwm.^2,pwm,ones(length(pwm),1)]*cTP2;

figure('Name','Eindhoven PWM-thrust curve');
hold on;
for i = 1:4
    plot(pwm,TP(:,i),'Color',c(i,:));
end
plot(pwm,TPAvg,'Color',c(5,:));
plot(pwm,TP2,'Color',c(6,:));
legend('Motor 1','Motor 2','Motor 3','Motor 4','Motor avg',...
       'Inverse relation');
xlabel('PWM (-)');
ylabel('Thrust (N)');
title('Eindhoven PWM-thrust curve');
