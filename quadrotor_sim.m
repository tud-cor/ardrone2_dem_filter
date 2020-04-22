%% TODO
%1. Derive linear system and implement

%2. Potentially add drag, see:
%-- "Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor"
%-- "Quadrotors and Accelerometers: State Estimation with an Improved Dynamic Model"
%-- "Nonlinear Dynamic Modeling for High Performance Control of a Quadrotor"

%% Initialisation
clc;
clear;
close all;


%% Load data
%TODO


%% System parameters
% Environmental constants
param.g             = 9.81;     %m/s^2
param.densityAir    = 1.2;      %kg/m^3 (for room temperature ~20 degree Celcius)

% Mass and inertia
%                   |ixx        ixy         ixz         |
% Inertia matrix:   |iyx = ixy  iyy         iyz         |
%                   |izx = ixz  izy = iyz   izz         |
% Rotor inertia only has izz component
param.m             = 1.477;    %kg
param.ixx           = 0.01152;  %kgm^2
param.ixy           = 0;        %kgm^2
param.ixz           = 0;        %kgm^2
param.iyy           = 0.01152;  %kgm^2
param.iyz           = 0;        %kgm^2
param.izz           = 0.0218;   %kgm^2
param.irotor        = 0.0000202;%kgm^2 TODO: value taken from Q. Li (2014)

% Dimensions
param.l             = 0.18;     %m TODO: average between Q. Li (2014) and measured using Blender
param.radiusBlade	= 0.096;    %m TODO: average between Q. Li (2014) and measured using Blender
param.areaBlade     = 2*pi*param.radiusBlade^2;   %m^2

% Thrust and torque coefficients
param.CT            = 0.008;        %TODO: optimise for this value using sim/flights (now derived from Bouabdallah using above numbers)
                                    %This value is 0.1225 in "Nonlinear control of quadrotor for point-tracking"
param.CQ            = 0.002;        %TODO: optimise for this value using sim/flights (now derived from Bouabdallah using above numbers)
                                    %This value is 0.0510 in "Nonlinear control of quadrotor for point-tracking"
param.cT            = param.CT*param.densityAir*param.areaBlade^2*param.radiusBlade^2;  %Ns^2/rad^2 (cT = F/omega^2)
param.cQ            = param.CQ*param.densityAir*param.areaBlade^2*param.radiusBlade^3;  %Nms^2/rad^2 (cQ = tau/omega^2)


%% LTI discrete-time simualation
% LTI system matrices
%TODO


%% (Nonlinear) ODE45 simulation
% States are based on "Design and control of an indoor micro quadrotor" -
% not anymore
tspan       = [0 10];
x0Nonlin    = zeros(12,1);
u           = kron(150, ones(1,4));

[t,xNonlin] = ode45(@(t,xNonlin) qrodefcn(xNonlin,u,param), tspan, x0Nonlin);

%% Plot results
%TODO
% plot(t,xNonlin(:,2));


%% Function definitions
function dxdt = qrodefcn(state,input,param)
%qrodefun ODE implementation of quadrotor model
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
R           = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);...
               cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);...
               -sin(theta)        , sin(phi)*cos(theta)                             , cos(phi)*cos(theta)                                 ];

Rr          = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
               0, cos(phi)           , -sin(phi);...
               0, sin(phi)/cos(theta), cos(phi)/cos(theta)    ];

dxdt = [R(1,1)*u + R(1,2)*v + R(1,3)*w;...
        R(2,1)*u + R(2,2)*v + R(2,3)*w;...
        R(3,1)*u + R(3,2)*v + R(3,3)*w;...
        r*v - q*w - param.g*sin(theta);...
        p*w - r*u + param.g*cos(theta)*sin(phi);...
        q*u - p*v + param.g*cos(theta)*cos(phi) + 1/param.m*param.cQ*T;...
        Rr(1,1)*p + Rr(1,2)*q + Rr(1,3)*r;...
        Rr(2,1)*p + Rr(2,2)*q + Rr(2,3)*r;...
        Rr(3,1)*p + Rr(3,2)*q + Rr(3,3)*r;...
        1/param.ixx*((param.iyy - param.izz)*q*r - param.irotor*q*omega + tauPhi);...
        1/param.iyy*((param.izz - param.ixx)*p*r + param.irotor*p*omega + tauTheta);...
        1/param.izz*((param.ixx - param.iyy)*p*q + tauPsi)                             ];

end
