%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dataset function
%
% Function to obtain pre-processed data from a .mat file, based on
% experimental AR.Drone 2.0 quadrotor flight data.
% 
% Author:          Dennis Benders, TU Delft, CoR
% Last modified:   09.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [t,ts,u,x,y,A,B,C,s,Pw,Pz] = get_ardrone2_flight_data

% Load experimental data
load ardrone2FlightData_wind2_tFrame4_yPhi_hPhiDot_cTEs2_zSigmaReport ...
     t ts uLin xLin yLin zPi wPi s A B C;

% Make data sizes compatible with DEM filter code
u = uLin;
x = xLin';
y = yLin';

% Set noise properties
s = 0.005;
Pw = wPi;
Pz = zPi;

end
