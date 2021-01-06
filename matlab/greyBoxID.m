%% Initialization
clear;
close all;
clc;

% Select grey-box identification method
% 0: greyest (for estimating complex relationships between unknown
%             variables in linear/nonlinear models)
% 1: ssest   (for estimating matrix elements in a structured matrix system
%             description)
select = 0;


%% Select experiment data
% % Load pre-processed experiment data file
% load expData10_29_2.mat expData;
% 
% % Select samples
% startSample = 1;
% endSample = 1800;
% 
% % y = [expData.output.otPos(2,startSample:endSample);...
% %      expData.output.otOrient(3,startSample:endSample)]';
% y = expData.output.otPos(3,startSample:endSample)' - ...
%     mean(expData.output.otPos(3,startSample:endSample));
% 
% u = expData.input.navMotor(:,startSample:endSample)';
% 
% ts = expData.sampleTime;
% 
% data = iddata(y,u,ts);


%% Select hovering data
load hoverBatA1.mat;
fs = 120;
ts = 1/fs;
data.time  = t;
data.value = [ardroneNavMotor;ardroneNavAltd];

tmp   = interpolate(ts,data);
motor = tmp.value(1:4,:);
altd  = tmp.value(5,:)/1000;

startSample = 820;
endSample = 4260;

y = altd(startSample:endSample)' - mean(altd(startSample:endSample));

u = motor(:,startSample:endSample)';

data = iddata(y,u,ts);

% t = tmp.time(startSample:endSample);
% 
% % Shift u forward
% s = 0;
% if s > 0
%     y = y(s+1:end);
%     u = u(1:end-s,:);
%     t = t(1:end-s);
% else
%     y = y(1:end+s);
%     u = u(-s+1:end,:);
%     t = t(1:end+s);
% end
% 
% cTPEs = [2.69656325287645e-05;
%          0.00243270089457810;
%          -2.77555756156289e-17];
% plot(t,y);
% hold on;
% pwm = sum(u,2)-4*171.4937;
% T = [pwm.^2,pwm,ones(length(pwm),1)]*cTPEs;
% plot(t,T);
% yline(0);


%% Set system parameters
% Fixed parameters
nu = 4;
nx = 2;
ny = 1;

g = 9.81;
l = 0.178;
% m = 0.481;
m = 0.497;
cM1 = 3.7;
cM2 = 130.9;
cA1 = cM1/2.55;
cA2 = cM2;
pwmEq = 170;

% Varying parameters
% ixx = 3.4e-3;
% iyy = 4.0e-3;
% izz = 6.9e-3;
ixx = 3.5e-3;
iyy = 4.1e-3;
izz = 6.9e-3;
cT1 = 8.6e-06;
cT2 = -3.2e-4;
cQ1 = 2.4e-7;
cQ2 = -9.9e-6;

% Set inertia minimum and maximum
ixxMin = 1e-3;
ixxMax = 1e-2;
iyyMin = 1e-3;
iyyMax = 1e-2;
izzMin = 1e-3;
izzMax = 1e-2;

% Set thrust coefficients minimum and maximum
% cT1Min = 1e-8;
% cT1Max = 1e-4;
% cT2Min = -1e-4;
% cT2Max = -1e-6;

cT1Min = -100;
cT1Max = 100;
cT2Min = -100;
cT2Max = 100;


%% Get identified model
if ~select
    % Create idgrey object
    odeFcn = 'greyBoxIDFcn';
    varParam = {'cT1',cT1;'cT2',cT2};
    fcnType = 'c';
    fixedParam = {nu,nx,ny,g,l,m,ixx,iyy,izz,cA1,cA2,pwmEq,cQ1,cQ2};
    estModel = idgrey(odeFcn,varParam,fcnType,fixedParam);

    % Set inertia constraints
%     estModel.Structure.Parameters(1).Minimum = ixxMin;
%     estModel.Structure.Parameters(1).Maximum = ixxMax;
%     estModel.Structure.Parameters(2).Minimum = iyyMin;
%     estModel.Structure.Parameters(2).Maximum = iyyMax;
%     estModel.Structure.Parameters(3).Minimum = izzMin;
%     estModel.Structure.Parameters(3).Maximum = izzMax;

    % Set thrust coefficients constraints
    estModel.Structure.Parameters(1).Minimum = cT1Min;
    estModel.Structure.Parameters(1).Maximum = cT1Max;
    estModel.Structure.Parameters(2).Minimum = cT2Min;
    estModel.Structure.Parameters(2).Maximum = cT2Max;

    % Perform grey-box parameter estimation
    greyestOpt = greyestOptions;
    greyestOpt.InitialState = 'estimate';
    greyestOpt.DisturbanceModel = 'estimate';
    greyestOpt.Focus = 'prediction';
    greyestOpt.EnforceStability = false;
    greyestOpt.EstimateCovariance = true;
%     greyestOpt.SearchMethod = 'fmincon';
    sysGB = greyest(data,estModel,greyestOpt);
    syscGB = ss(sysGB.A,sysGB.B,sysGB.C,sysGB.D);
    paramEst = getpvec(sysGB);
    paramCov = getcov(sysGB);
else
    % Set system matrices
    A = [0, 0, 0, 1, 0, 0, 0 , 0, 0, 0, 0, 0;
         0, 0, 0, 0, 1, 0, 0 , 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 1, 0 , 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0, 0 , g, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0, -g, 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0, 0 , 0, 0, 1, 0, 0;
         0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 1, 0;
         0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 1;
         0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0];
    B = [0  , 0    , 0    , 0;
         0  , 0    , 0    , 0;
         0  , 0    , 0    , 0;
         0  , 0    , 0    , 0;
         0  , 0    , 0    , 0;
         1/m, 0    , 0    , 0;
         0  , 0    , 0    , 0;
         0  , 0    , 0    , 0;
         0  , 0    , 0    , 0;
         0  , 1/ixx, 0    , 0;
         0  , 0    , 1/iyy, 0;
         0  , 0    , 0    , 1/izz];
    C = zeros(ny,nx);
    C(1:3,1:3) = eye(3);
    C(4:6,7:9) = eye(3);
    D = zeros(ny,nu);
    K = zeros(nx,ny);
    x0 = zeros(nx,1);

    % Set free parameters
    estModel = idss(A,B,C,D,'Ts',0);
    estModel.Structure.A.Free = false;
    estModel.Structure.B.Free(:,:) = false;
    estModel.Structure.C.Free(:,:) = false;
    estModel.Structure.D.Free(:,:) = false;
    estModel.Structure.K.Free(:,:) = true;
    % TODO: add estModel.noiseVariance here!

    % Set mass constraints
%     estModel.Structure.B.Free(6,1) = true;
%     estModel.Structure.B.Minimum(6,1) = 1/mMax;
%     estModel.Structure.B.Maximum(6,1) = 1/mMin;

    % Set inertia constraints
    estModel.Structure.B.Free(10,2) = true;
    estModel.Structure.B.Minimum(10,2) = 1/ixxMax;
    estModel.Structure.B.Maximum(10,2) = 1/ixxMin;
    estModel.Structure.B.Free(11,3) = true;
    estModel.Structure.B.Minimum(11,3) = 1/iyyMax;
    estModel.Structure.B.Maximum(11,3) = 1/iyyMin;
    estModel.Structure.B.Free(12,4) = true;
    estModel.Structure.B.Minimum(12,4) = 1/izzMax;
    estModel.Structure.B.Maximum(12,4) = 1/izzMin;

    % Set estimation options
    ssestOpt = ssestOptions;
    ssestOpt.InitialState = idpar(x0);
    ssestOpt.InitialState.Free(:) = true;

    % Get estimated model
    sysGB = ssest(data,estModel,ssestOpt);
    syscGB = ss(sysGB.A,sysGB.B,sysGB.C,sysGB.D);
end


%% Show estimated thrust characteristic
load thrust_torque_comp_theses.mat;
omegaR = linspace(0,500)';

cT1Est = paramEst(1);
cT2Est = paramEst(2);

% Thrust
Tq = omegaR.^2*thrustPoly.p1;
T2wc = [omegaR.^2,omegaR]*thrustPoly.p2;
T2 = [omegaR.^2,omegaR,ones(length(omegaR),1)]*thrustPoly.p3;
TEst = [omegaR.^2,omegaR]*[cT1Est;cT2Est];

T2Lin = [2*omegaR,ones(length(omegaR),1)]*...
        [thrustPoly.p3(1:2,:),[cT1Est;cT2Est]];

figure('Name','Nonlinear thrust curves');
plot(omegaR,[T2wc(:,1),T2(:,2:3),TEst],'-o');
legend('Own work','Delft thesis','Eindhoven thesis','Estimated',...
       'location','northwest');
figure('Name','Linearized thrust curves');
plot(omegaR,T2Lin);
legend('Own work','Delft thesis','Eindhoven thesis','Estimated',...
       'location','northwest');


%% Save data
filename = sprintf('sysGB_exp_24-7_7_%s.mat',...
                   datestr(now,'dd-mm-yyyy_HH-MM'));
% save(filename,'syscGB');
