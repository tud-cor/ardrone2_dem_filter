%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Quadrotor simulation
%
% This file loads simulation data, specifies quadrotor parameters,
% simulates the mathematical quadrotor model and plots the results in order
% to compare with physical flight (or Gazebo simulation) data
%
% Author: Dennis Benders
% Last edited: 13.05.2020
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialisation
clc;
clear;
close all;


%% Select simulation data
% Load pre-processed simulation data file
load bagdata_15-09-2020_15-05.mat expData;

startSample = 1;
endSample = 10000;


%% System parameters
% Floating-point accuracy
param.timeThres     = 1e-10;

% Position at the ground accuracy
param.groundThres   = 1e-3;     %m

% Recorded data accuracy
param.sampleTime    = expData.sampleTime;   %s

% Environmental constants
param.g             = 9.81;     %m/s^2
param.densityAir    = 1.2;      %kg/m^3 (for room temperature
                                %        ~20 degree Celcius)

% Mass and inertia
%                   |ixx        ixy         ixz         |
% Inertia matrix:   |iyx = ixy  iyy         iyz         |
%                   |izx = ixz  izy = iyz   izz         |
% Rotor inertia only has izz component
param.m             = 0.481;	%kg
param.ixx           = 3.4e-3;	%kgm^2
param.ixy           = 0;        %kgm^2
param.ixz           = 0;        %kgm^2
param.iyy           = 4.0e-3;   %kgm^2
param.iyz           = 0;        %kgm^2
param.izz           = 6.9e-3;   %kgm^2
param.irotor        = 2.030e-5;	%kgm^2 TODO: value from Q. Li (2014)

% Dimensions
% %TODO: average between Q. Li (2014) and measured using Blender
% param.l             = 0.18;     %m
% %TODO: average between Q. Li (2014) and measured using Blender
% param.radiusBlade	= 0.096;    %m
% param.areaBlade     = 2*pi*param.radiusBlade^2;   %m^2
param.l             = 0.178;	%m

% Thrust and torque coefficients
% %TODO: optimise for this value using sim/flights
% %(now derived from Bouabdallah using their numbers for radius, area, etc.)
% %This value is 0.1225 in Nonlinear control of quadrotor for point-tracking
% param.CT            = 0.008;
% 
% %TODO: optimise for this value using sim/flights
% %(now derived from Bouabdallah using their numbers for radius, area, etc.)
% %This value is 0.0510 in Nonlinear control of quadrotor for point-tracking
% param.CQ            = 0.001;
% 
% param.cT            = param.CT*param.densityAir*param.areaBlade* ...
%                       param.radiusBlade^2;  %Ns^2/rad^2 (cT = F/omega^2)
% param.cQ            = param.CQ*param.densityAir*param.areaBlade^2* ...
%                       param.radiusBlade^3;  %Nms^2/rad^2 (cQ = tau/omega^2)
%PWM-PWM relation: PWM_ardrone/navdata = 2.55*PWM_toolbox
param.PwmToPwm      = 2.55;
% omegaR = PwmToOmegaR(1)*pwm + PwmToOmegaR(2)
param.PwmToOmegaR   = [3.7,130.9];
% cT(1)*omegaR^2 + cT*omegaR
param.cT            = [8.6e-6,-3.2e-4];
% cQ(1)*omegaR^2 + cq*omegaR
param.cQ            = [2.4e-7,-9.9e-6];


%% LTI state-space description and discretize
% Construct continuous-time linearised state space system
n = 12;
l = 4;
A = [0, 0, 0, 1, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 1, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , param.g, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, -param.g, 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 1, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0       , 0      , 0, 0, 0, 0];
B = [0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     1/param.m, 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 0          , 0          , 0;
     0        , 1/param.ixx, 0          , 0;
     0        , 0          , 1/param.iyy, 0;
     0        , 0          , 0          , 1/param.izz];
C = eye(n);
D = zeros(n,l);

sysc = ss(A,B,C,D);

% Construct discrete-time linearised state space system
sysd = c2d(sysc,param.sampleTime);


%% Simulation parameters
t           = expData.input.time(startSample:endSample);
x0          = [expData.state.otPos(:,1);zeros(3,1);...
               expData.state.otOrient(:,1);zeros(3,1)];
% x0          = zeros(12,1);
% x0(3)       = 1;

% Construct rotor speed input
rotorSpeed  = expData.input.motor;
% rotorSpeed  = kron(ones(size(expData.input.motor,1),...
%                         size(expData.input.motor,2)),...
%                    mean(mean(expData.input.motor)));

% Construct input from rotor speeds
pwmToolbox  = rotorSpeed/param.PwmToPwm;
omegaR      = param.PwmToOmegaR(1)*pwmToolbox+param.PwmToOmegaR(2);
dur         = length(expData.input.time(startSample:endSample));
f           = zeros(4,1);
T           = zeros(1,dur);
tauPhi      = zeros(1,dur);
tauTheta    = zeros(1,dur);
tauPsi      = zeros(1,dur);
for i = 1:dur
    f           = param.cT(1)*omegaR(:,i).^2 + param.cT(2)*omegaR(:,i);
%     f           = 9e-6*omegaR(:,i).^2 - 5.6e-4*omegaR(:,i) + 3.2e-2;
%     f           = 7.7e-6*omegaR(:,i).^2;

%     f(1)        = 1.5618e-4*pwmToolbox(1,i)^2 + ...
%                   1.0395e-2*pwmToolbox(1,i) + 0.13894;
%     f(2)        = 1.8150e-4*pwmToolbox(2,i)^2 + ...
%                   8.7242e-3*pwmToolbox(2,i) + 0.14425;
%     f(3)        = 1.3478e-4*pwmToolbox(3,i)^2 + ...
%                   7.3295e-3*pwmToolbox(3,i) + 0.11698;
%     f(4)        = 1.4306e-4*pwmToolbox(4,i)^2 + ...
%                   5.7609e-3*pwmToolbox(4,i) + 0.13362;

%     f           = 8.386e-6*omegaR(:,i).^2 - 3.723e-5*omegaR(:,i) - 0.0318;
    T(i)        = sum(f);
    tauPhi(i)   = sqrt(1/2)*param.l*(f(1)-f(2)-f(3)+f(4));
    tauTheta(i) = sqrt(1/2)*param.l*(-f(1)-f(2)+f(3)+f(4));
    tauPsi(i)   = param.cQ(1)*(sum(omegaR(1:2:3,i).^2)-...
                               sum(omegaR(2:2:4).^2)) + ...
                  param.cQ(2)*(sum(omegaR(1:2:3,i))-sum(omegaR(2:2:4)));
end
u = [T;tauPhi;tauTheta;tauPsi];



% Custom input
% Correctly working when looking at position data
% u       = kron(ones(1,length(t)-1),[0; 0; 0; 0]);
% u       = kron(ones(1,length(t)),[param.m*param.g; 0; 0; 0]);

% Not tested yet
% uFreq   = 5; %Hz
% u       = kron(ones(1,dur/4),[3; 0; 0; 0]);
% u       = [u, kron(ones(1,3*dur/4),[0; 0; 0; 0])];
% for i = 1:dur
%     u(2,i) = 0.001*cos(2*pi*uFreq*i*param.sampleTime);
% end


%% Simulate LTI system (using manual iterations)
x = ltiSim(sysd,t,x0,u,param);


%% Simulate LTI system (using MATLAB function lsim)
[tLsim,xLsim] = ltiLsim(sysd,t,x0,u,param);


%% TODO: simulate nonlinear system model


%% Plot results and compare with OptiTrack data
figure('Name','Position');
subplot(3,1,1);
hold on;
plot(t,x(1,:));
plot(tLsim,xLsim(1,1:end-1));
plot(t,expData.state.otPos(1,startSample:endSample));
legend('LTI simulation','lsim','OptiTrack');
title('x');
subplot(3,1,2);
hold on;
plot(t,x(2,:));
plot(tLsim,xLsim(2,1:end-1));
plot(t,expData.state.otPos(2,startSample:endSample));
legend('LtI simulation','lsim','OptiTrack');
title('y');
subplot(3,1,3);
hold on;
plot(t,x(3,:));
plot(tLsim,xLsim(3,1:end-1));
plot(t,expData.state.otPos(3,startSample:endSample));
legend('LtI simulation','lsim','OptiTrack');
title('z');

% figure('Name','Velocity');
% subplot(3,1,1);
% plot(t,x(4,:));
% title('v_x');
% subplot(3,1,2);
% plot(t,x(5,:));
% title('v_y');
% subplot(3,1,3);
% plot(t,x(6,:),'-o');
% title('v_z');


%% Nonlinear (non-working) simulations
% TODO: not correct yet!
% (Nonlinear) ODE45 simulation
% States are based on "Design and control of an indoor micro quadrotor" -
% not anymore
% tspan       = [0,1000];
% x0Nonlin    = zeros(1,12);
% u           = kron(840.8658965145,ones(1,4)); %hovering: u = 840.8658965145
%
% % opt = odeset('AbsTol', 1e-20);
% [t,xNonlin] = ode45(@(t,xNonlin) qrodefcn(xNonlin,u,param),tspan,x0Nonlin);


% Gazebo simulation comparison
% TODO: not correct yet!
% States are based on "Design and control of an indoor micro quadrotor" -
% not anymore
% endSample   = 200;
% time        = gazSim.input.time(startSample:endSample);
% x0Nonlin	= zeros(12,1);
% x0Nonlin(3) = 0.4;
%
% opt         = odeset('Stats', 'on');
% [t,xNonlin] = ode45(@(t,xNonlin) qrgazodefcn(t,xNonlin,param),time,x0Nonlin,opt);


% %% Plot results
% % Determine plot limits
% xMin        = min(min(state(1,startSample:endSample)), ...
%                   min(gazSim.state.pos(1,startSample:endSample)));
% xMax        = max(max(state(1,startSample:endSample)), ...
%                   max(gazSim.state.pos(1,startSample:endSample)));
% yMin        = min(min(state(2,startSample:endSample)), ...
%                   min(gazSim.state.pos(2,startSample:endSample)));
% yMax        = max(max(state(2,startSample:endSample)), ...
%                   max(gazSim.state.pos(2,startSample:endSample)));
% zMin        = min(min(state(3,startSample:endSample)), ...
%                   min(gazSim.state.pos(3,startSample:endSample)));
% zMax        = max(max(state(3,startSample:endSample)), ...
%                   max(gazSim.state.pos(3,startSample:endSample)));
% phiMin      = min(min(state(7,startSample:endSample)), ...
%                   min(gazSim.state.orient(1,startSample:endSample)));
% phiMax      = max(max(state(7,startSample:endSample)), ...
%                   max(gazSim.state.orient(1,startSample:endSample)));
% thetaMin    = min(min(state(8,startSample:endSample)), ...
%                   min(gazSim.state.orient(2,startSample:endSample)));
% thetaMax    = max(max(state(8,startSample:endSample)), ...
%                   max(gazSim.state.orient(2,startSample:endSample)));
% psiMin      = min(min(state(9,startSample:endSample)), ...
%                   min(gazSim.state.orient(3,startSample:endSample)));
% psiMax      = max(max(state(9,startSample:endSample)), ...
%                   max(gazSim.state.orient(3,startSample:endSample)));
%
% % Plot position
% figure('Name', 'Position in simulation plots (inertial frame)');
%
% subplot(3,2,1);
% plot(time,state(1,:));
% xlim([time(1),time(end)]);
% ylim([xMin,xMax]);
% xlabel('Time (s)');
% ylabel('x_{matlab} (m)');
% subplot(3,2,2);
% plot(gazSim.state.time,gazSim.state.pos(1,:));
% xlim([time(1),time(end)]);
% % ylim([xMin xMax]);
% xlabel('Time (s)');
% ylabel('x_{gazebo} (m)');
%
% subplot(3,2,3);
% plot(time,state(2,:));
% xlim([time(1),time(end)]);
% ylim([yMin,yMax]);
% xlabel('Time (s)');
% ylabel('y_{matlab} (m)');
% subplot(3,2,4);
% plot(gazSim.state.time,gazSim.state.pos(2,:));
% xlim([time(1),time(end)]);
% % ylim([yMin,yMax]);
% xlabel('Time (s)');
% ylabel('y_{gazebo} (m)');
%
% subplot(3,2,5);
% plot(time,state(3,:));
% xlim([time(1),time(end)]);
% ylim([zMin,zMax]);
% xlabel('Time (s)');
% ylabel('z_{matlab} (m)');
% subplot(3,2,6);
% plot(gazSim.state.time,gazSim.state.pos(3,:));
% xlim([time(1),time(end)]);
% % ylim([zMin,zMax]);
% xlabel('Time (s)');
% ylabel('z_{gazebo} (m)');
%
% % Plot XYZ fixed angles/ZYX Euler angles
% figure('Name', 'Angle plots (XYZ fixed/ZYX Euler)');
% subplot(3,2,1);
% plot(time,state(4,:));
% xlim([time(1),time(end)]);
% ylim([phiMin,phiMax]);
% xlabel('Time (s)');
% ylabel('\phi_{matlab} (rad)');
% subplot(3,2,2);
% plot(gazSim.state.time,gazSim.state.orient(1,:));
% xlim([time(1),time(end)]);
% % ylim([phiMin,phiMax]);
% xlabel('Time (s)');
% ylabel('\phi_{gazebo} (rad)');
%
% subplot(3,2,3);
% plot(time,state(5,:));
% xlim([time(1),time(end)]);
% ylim([thetaMin,thetaMax]);
% xlabel('Time (s)');
% ylabel('\theta (rad)');
% subplot(3,2,4);
% plot(gazSim.state.time,gazSim.state.orient(2,:));
% xlim([time(1),time(end)]);
% % ylim([thetaMin,thetaMax]);
% xlabel('Time (s)');
% ylabel('\theta_{gazebo} (rad)');
%
% subplot(3,2,5);
% plot(time,state(6,:));
% xlim([time(1),time(end)]);
% ylim([psiMin,psiMax]);
% xlabel('Time (s)');
% ylabel('\psi (rad)');
% subplot(3,2,6);
% plot(gazSim.state.time,gazSim.state.orient(3,:));
% xlim([time(1),time(end)]);
% % ylim([psiMin,psiMax]);
% xlabel('Time (s)');
% ylabel('\psi_{gazebo} (rad)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure('Name','States');
% subplot(6,2,1);
% plot(time,state(1,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('x_{matlab} (m)');
% 
% subplot(6,2,3);
% plot(time,state(2,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('y_{matlab} (m)');
% 
% subplot(6,2,5);
% plot(time,state(3,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('z_{matlab} (m)');
% 
% subplot(6,2,7);
% plot(time,state(4,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('xDot_{matlab} (m)');
% 
% subplot(6,2,9);
% plot(time,state(5,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('yDot_{matlab} (m)');
% 
% subplot(6,2,11);
% plot(time,state(6,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('zDot_{matlab} (m)');
% 
% % Plot XYZ fixed angles/ZYX Euler angles
% subplot(6,2,2);
% plot(time,state(7,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('\phi_{matlab} (rad)');
% 
% subplot(6,2,4);
% plot(time,state(8,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('\theta_{matlab} (rad)');
% 
% subplot(6,2,6);
% plot(time,state(9,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('\psi_{matlab} (rad)');
% 
% subplot(6,2,8);
% plot(time,state(10,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('\phiDot_{matlab} (rad)');
% 
% subplot(6,2,10);
% plot(time,state(11,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('\thetaDot_{matlab} (rad)');
% 
% subplot(6,2,12);
% plot(time,state(12,:));
% xlim([time(1),time(end)]);
% xlabel('Time (s)');
% ylabel('\psiDot_{matlab} (rad)');


%% Function definitions
function dxdt = qrodefcn(state,input,param)
%QRODEFUN ODE implementation of quadrotor model.
%   This function provides the ODEs of the quadrotor model.
%   The ODEs used in this function contain the most (nonlinear) terms as
%   defined in the quadrotor model.

% State definition:
x       = state(1);     %x position in inertial frame
y       = state(2);     %y position in inertial frame
z       = state(3);     %z position in inertial frame
u       = state(4);     %linear velocity along x-axis in body frame
v       = state(5);     %linear velocity along y-axis in body frame
w       = state(6);     %linear velocity along z-axis in body frame
phi     = state(7);     %roll Euler angle
theta   = state(8);     %pitch Euler angle
psi     = state(9);     %yaw Euler angle
p       = state(10);    %angular velocity around x-axis in body frame
q       = state(11);    %angular velocity around y-axis in body frame
r       = state(12);    %angular velocity around z-axis in body frame

% Input definition:
omega1  = input(1);     %rotational velocity of rotor 1
omega2  = input(2);     %rotational velocity of rotor 2
omega3  = input(3);     %rotational velocity of rotor 3
omega4  = input(4);     %rotational velocity of rotor 4

% Thrust and torque values
T           = param.cT*(omega1^2 + omega2^2 + omega3^2 + omega4^2);
tauPhi      = param.cT*(omega4^2 - omega2^2);
tauTheta    = param.cT*(omega3^2 - omega1^2);
tauPsi      = param.cQ*(omega2^2 + omega4^2 - omega1^2 - omega3^2);
omega       = omega2 + omega4 - omega1 - omega3;

% Linear and angular rotation matrices (constructed using ZYX Euler
% angles):
R           = [cos(theta)*cos(psi), ...
               sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), ...
               cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);...
               cos(theta)*sin(psi), ...
               sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), ...
               cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);...
               -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

Rr          = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
               0, cos(phi)           , -sin(phi);...
               0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

dxdt = [R(1,1)*u + R(1,2)*v + R(1,3)*w;...
        R(2,1)*u + R(2,2)*v + R(2,3)*w;...
        R(3,1)*u + R(3,2)*v + R(3,3)*w;...
        r*v - q*w + param.g*sin(theta);...
        p*w - r*u - param.g*cos(theta)*sin(phi);...
        q*u - p*v - param.g*cos(theta)*cos(phi) + 1/param.m*T;...
        Rr(1,1)*p + Rr(1,2)*q + Rr(1,3)*r;...
        Rr(2,1)*p + Rr(2,2)*q + Rr(2,3)*r;...
        Rr(3,1)*p + Rr(3,2)*q + Rr(3,3)*r;...
        1/param.ixx*((param.iyy - param.izz)*q*r - ...
        param.irotor*q*omega + tauPhi);...
        1/param.iyy*((param.izz - param.ixx)*p*r + ...
        param.irotor*p*omega + tauTheta);...
        1/param.izz*((param.ixx - param.iyy)*p*q + tauPsi)];
end

function dxdt = qrgazodefcn(t,state,param)
%QRODEFUN ODE implementation of quadrotor model.
%   This function provides the ODEs of the quadrotor model.
%   The ODEs used in this function contain the most (nonlinear) terms as
%   defined in the quadrotor model.

% State definition:
x       = state(1);     %x position in inertial frame
y       = state(2);     %y position in inertial frame
z       = state(3);     %z position in inertial frame
u       = state(4);     %linear velocity along x-axis in body frame
v       = state(5);     %linear velocity along y-axis in body frame
w       = state(6);     %linear velocity along z-axis in body frame
phi     = state(7);     %roll Euler angle
theta   = state(8);     %pitch Euler angle
psi     = state(9);     %yaw Euler angle
p       = state(10);    %angular velocity around x-axis in body frame
q       = state(11);    %angular velocity around y-axis in body frame
r       = state(12);    %angular velocity around z-axis in body frame

% Load data and determine time index in input arrays
load gazSim.mat gazSim;
for i = 1:length(gazSim.input.time)
    if abs(gazSim.input.time(i) - t) < param.timeThres
        break;
    elseif t >= gazSim.input.time(i) && t <= gazSim.input.time(i+1) && ...
           abs(gazSim.input.time(i) - t) <= abs(gazSim.input.time(i+1) - t)
        break;
    elseif t >= gazSim.input.time(i) && t <= gazSim.input.time(i+1) && ...
           abs(gazSim.input.time(i) - t) > abs(gazSim.input.time(i+1) - t)
        i = i + 1;
        break;
    end
end

% Thrust and torque values
T           = gazSim.input.force(3,i);
tauPhi      = gazSim.input.torque(1,i);
tauTheta    = gazSim.input.torque(2,i);
tauPsi      = gazSim.input.torque(3,i);

% Linear and angular rotation matrices (constructed using ZYX Euler
% angles):
R           = [cos(theta)*cos(psi), ...
               sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), ...
               cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);...
               cos(theta)*sin(psi), ...
               sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), ...
               cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);...
               -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

Rr          = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
               0, cos(phi)           , -sin(phi);...
               0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

dxdt = [R(1,1)*u + R(1,2)*v + R(1,3)*w;...
        R(2,1)*u + R(2,2)*v + R(2,3)*w;...
        R(3,1)*u + R(3,2)*v + R(3,3)*w;...
        r*v - q*w + param.g*sin(theta);...
        p*w - r*u - param.g*cos(theta)*sin(phi);...
        q*u - p*v - param.g*cos(theta)*cos(phi) + 1/param.m*T;...
        Rr(1,1)*p + Rr(1,2)*q + Rr(1,3)*r;...
        Rr(2,1)*p + Rr(2,2)*q + Rr(2,3)*r;...
        Rr(3,1)*p + Rr(3,2)*q + Rr(3,3)*r;...
        1/param.ixx*((param.iyy - param.izz)*q*r - tauPhi);...
        1/param.iyy*((param.izz - param.ixx)*p*r + tauTheta);...
        1/param.izz*((param.ixx - param.iyy)*p*q + tauPsi)];

% Ground constraint
% if state(3) < 0
%     state(3) = 0;
% end
% if abs(state(3)) < param.groundThres && dxdt(3) < 0
%     dxdt(3) = 0;
% end
end
