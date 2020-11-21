%% Initialization
clear;
close all;
clc;


%% Load generalized output and measurement noise data
p = 6;

filename = ['deriveSmoothnessFromPrecisionP' num2str(p) 'S5e-3'];
load(filename);


%% Calculate covariance matrix of measurement signal
zSigma = inv(zPi);


%% Calculate precision matrix from generalized output
if p == 2
zSigmaTildeEst = getCov3x3(yTilde);
elseif p == 3
    zSigmaTildeEst = getCov4x4(yTilde);
elseif p == 6
    zSigmaTildeEst = getCov7x7(yTilde);
end
zSigmaEst = zSigmaTildeEst(1,1);


%% Calculate smoothness from covariance matrix generalized output,
%  assuming the real measurement standard deviation
sEst = zeros(p+1,p);

% Using (2,2) element (1st term)
sEst(1,1) = sqrt(zSigma/(2*zSigmaTildeEst(2,2)));

% Using (1,3) element (1st term)
sEst(2,1) = sqrt(-zSigma/(2*zSigmaTildeEst(1,3)));

% Using (3,1) element (1st term)
sEst(3,1) = sqrt(-zSigma/(2*zSigmaTildeEst(3,1)));


% Using (3,3) element (2nd term)
sEst(1,2) = nthroot(3*zSigma/(4*zSigmaTildeEst(3,3)),4);

if p >= 3
    % Using (2,4) element (2nd term)
    sEst(2,2) = nthroot(-3*zSigma/(4*zSigmaTildeEst(2,4)),4);

    % Using (4,2) element (2nd term)
    sEst(3,2) = nthroot(-3*zSigma/(4*zSigmaTildeEst(4,2)),4);


    % Using (4,4) element (3rd term)
    sEst(1,3) = nthroot(15*zSigma/(8*zSigmaTildeEst(4,4)),6);


    if p >= 4
        % Using (1,5) element (2nd term)
        sEst(4,2) = nthroot(3*zSigma/(4*zSigmaTildeEst(1,5)),4);

        % Using (5,1) element (2nd term)
        sEst(5,2) = nthroot(3*zSigma/(4*zSigmaTildeEst(5,1)),4);


        % Using (3,5) element (3rd term)
        sEst(2,3) = nthroot(-15*zSigma/(8*zSigmaTildeEst(3,5)),6);

        % Using (5,3) element (3rd term)
        sEst(3,3) = nthroot(-15*zSigma/(8*zSigmaTildeEst(5,3)),6);


        % Using (5,5) element (4th term)
        sEst(1,4) = nthroot(105*zSigma/(16*zSigmaTildeEst(5,5)),8);


        if p >= 5
            % Using (2,6) element (3rd term)
            sEst(4,3) = nthroot(15*zSigma/(8*zSigmaTildeEst(2,6)),6);

            % Using (6,2) element (3rd term)
            sEst(5,3) = nthroot(15*zSigma/(8*zSigmaTildeEst(6,2)),6);


            % Using (4,6) element (4th term)
            sEst(2,4) = nthroot(-105*zSigma/(16*zSigmaTildeEst(4,6)),8);

            % Using (6,4) element (4th term)
            sEst(3,4) = nthroot(-105*zSigma/(16*zSigmaTildeEst(6,4)),8);


            % Using (6,6) element (5th term)
            sEst(1,5) = nthroot(945*zSigma/(32*zSigmaTildeEst(6,6)),10);


            if p >= 6
                % Using (1,7) element (3rd term)
                sEst(6,3) = nthroot(-15*zSigma/(8*zSigmaTildeEst(1,7)),6);

                % Using (7,1) element (3rd term)
                sEst(7,3) = nthroot(-15*zSigma/(8*zSigmaTildeEst(7,1)),6);


                % Using (3,7) element (4th term)
                sEst(4,4) = nthroot(105*zSigma/(16*zSigmaTildeEst(3,7)),8);

                % Using (7,3) element (4th term)
                sEst(5,4) = nthroot(105*zSigma/(16*zSigmaTildeEst(7,3)),8);


                % Using (5,7) element (5th term)
                sEst(2,5) = ...
                    nthroot(-945*zSigma/(32*zSigmaTildeEst(5,7)),10);

                % Using (7,5) element (5th term)
                sEst(3,5) = ...
                    nthroot(-945*zSigma/(32*zSigmaTildeEst(7,5)),10);


                % Using (7,7) element (6th term)
                sEst(1,6) = ...
                    nthroot(10395*zSigma/(64*zSigmaTildeEst(7,7)),12);
            end
        end
    end
end


% %% Calculate smoothness from covariance matrix generalized output,
% %  assuming the calculated measurement standard deviation
% % Using (2,2) element
% sEst2(1) = sqrt(zSigmaEst/(2*zSigmaTildeEst(2,2)));
% 
% % Using (1,3) element
% sEst2(2) = sqrt(-zSigmaEst/(2*zSigmaTildeEst(1,3)));
% 
% % Using (3,1) element
% sEst2(3) = sqrt(-zSigmaEst/(2*zSigmaTildeEst(3,1)));
% 
% % Using (3,3) element
% sEst2(4) = nthroot(3*zSigmaEst/(4*zSigmaTildeEst(3,3)),4);
% 
% if p >= 3
%     % Using (2,4) element
%     sEst2(5) = nthroot(-3*zSigmaEst/(4*zSigmaTildeEst(2,4)),4);
% 
%     % Using (4,2) element
%     sEst2(6) = nthroot(-3*zSigmaEst/(4*zSigmaTildeEst(4,2)),4);
% 
%     % Using (4,4) element
%     sEst2(7) = nthroot(15*zSigmaEst/(8*zSigmaTildeEst(4,4)),6);
% end
