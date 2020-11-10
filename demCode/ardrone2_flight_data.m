function [t,ts,u,x,y,Pz,Pw,s,A,B,C] = ardrone2_flight_data
% load ardrone2FlightDataZ_UZeroed.mat t ts uLin xLin yLin ySigma yS yS2 A B C;

% load ardrone2FlightData5.mat t ts uLin xLin yLin ySigma yS A B C;
% load ardrone2FlightData5_UZeroed.mat t ts uLin xLin yLin ySigma yS A B C;
% load ardrone2FlightData5_psiOffsetHalfPi.mat t ts uLin xLin yLin ySigma yS A B C;
% load ardrone2FlightData5_psiOffset-1-75.mat t ts uLin xLin yLin ySigma yS A B C;
% load ardrone2FlightData5_thetaDrone.mat t ts uLin xLin yLin ySigma yS A B C;

% u = uLin;
% x = xLin';
% y = yLin';
% sigma = ySigma;
% s = yS;


% load ardrone2FlightData6.mat t ts uLin xLin yLin s A B C;
% load ardrone2FlightData6_EindhovenCoef_PWMapart t ts uLin xLin yLin s A B C;
% load ardrone2FlightData6_timeFrameSelect t ts uLin xLin yLin zPi wPi s A B C;
% u = uLin;
% x = xLin';
% y = yLin';
% Pz = zPi;
% Pw = wPi;
% s = 0.01;


% load ardrone2FlightData7_wind2_hPhi t ts uLin xLin yLin zPi wPi s A B C;
% load ardrone2FlightData7_hover t ts uLin xLin yLin zPi wPi s A B C;
load ardrone2FlightData7_wind2_yPhi_hPhiDot t ts uLin xLin yLin zPi wPi s A B C;

% Working:
% load ardrone2FlightData7_wind2_hPhiDot t ts uLin xLin yLin zPi wPi s A B C;
u = uLin;
x = xLin';
y = yLin';
Pz = zPi;
% Pz = eye(2);
% Pz = [1e6,0;0,1e6];
Pw = wPi;
% Pw = eye(3);
% Pw = [1e8,0,0;0,1e8,0;0,0,1e8];
% s = mean(s);
% s = ts;
s = 0.005;

% For DEM approximating Kalman with a lot of generalized coordinates
% (p=6,d=6)
% u = uLin;
% x = xLin';
% y = yLin';
% Pz = zPi;
% Pw = eye(3);
% s = 0.0008;
end