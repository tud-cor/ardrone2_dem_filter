%% Initialization
clear;
close all;
clc;

% Select grey-box identification method
% 0: greyest
% 1: ssest
select = 1;


%% Select data
% Load pre-processed experiment data file
load hoverSpiralling25-100Hz15-120s.mat expData;

% Select samples
startSample = 1;
endSample = 2600;

pos    = expData.output.otPos(:,startSample:endSample);
orient = expData.output.otOrient(:,startSample:endSample);
y = [pos;orient]';

uMotor = expData.input.motor(:,startSample:endSample);
u = uMotor';

ts = expData.sampleTime;

data = iddata(y,u,ts);


%% Set system parameters
% Fixed parameters
g = 9.81;
nu = 4;
nx = 12;
ny = 6;

% Varying parameters
m = 0.481;
ixx = 3.4e-3;
iyy = 4.0e-3;
izz = 6.9e-3;

% Set mass minimum and maximum
mMin = 0.46;
mMax = 0.52;

% Set inertia minimum and maximum
ixxMin = 1e-3;
ixxMax = 1;
iyyMin = 1e-3;
iyyMax = 1;
izzMin = 1e-3;
izzMax = 1;


%% Get identified model
if ~select
    % Create idgrey object
    odeFcn = 'greyBoxIDFcn';
    varParam = {'m',m;'ixx',ixx;'iyy',iyy;'izz',izz};
    fcnType = 'c';
    fixedParam = {g,nu,nx,ny};
    estModel = idgrey(odeFcn,varParam,fcnType,fixedParam);

    % Set mass constraints
    estModel.Structure.Parameters(1).Minimum = 1/mMax;
    estModel.Structure.Parameters(1).Maximum = 1/mMin;

    % Set inertia constraints
    estModel.Structure.Parameters(2).Minimum = 1/ixxMax;
    estModel.Structure.Parameters(2).Maximum = 1/ixxMin;
    estModel.Structure.Parameters(3).Minimum = 1/iyyMax;
    estModel.Structure.Parameters(3).Maximum = 1/iyyMin;
    estModel.Structure.Parameters(4).Minimum = 1/izzMax;
    estModel.Structure.Parameters(4).Maximum = 1/izzMin;

    % Perform grey-box parameter estimation
    greyestOpt = greyestOptions;
    greyestOpt.InitialState = 'estimate';
    greyestOpt.DisturbanceModel = 'estimate';
    greyestOpt.EnforceStability = false;
    sysGB = greyest(data,estModel,greyestOpt);
    syscGB = ss(sysGB.A,sysGB.B,sysGB.C,sysGB.D);
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


%% Save data
save('sysGB_exp_24-7_7.mat','syscGB');
