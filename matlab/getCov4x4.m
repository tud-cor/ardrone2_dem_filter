%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Construct 4x4 covariance matrix
%
% Function to generate the values of each of the elements in the 4x4
% covariance matrix belonging to the input x.
% 
% Author:        Dennis Benders, TU Delft, CoR
% Last modified: 21.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xCov] = getCov4x4(x)

c = cov(x(1,:),x(2,:));
cX11 = c(1,1);
cX12 = c(1,2);
cX21 = c(2,1);
cX22 = c(2,2);

c = cov(x(1,:),x(3,:));
cX13 = c(1,2);
cX31 = c(2,1);
cX33 = c(2,2);

c = cov(x(1,:),x(4,:));
cX14 = c(1,2);
cX41 = c(2,1);
cX44 = c(2,2);

c = cov(x(2,:),x(3,:));
cX23 = c(1,2);
cX32 = c(2,1);

c = cov(x(2,:),x(4,:));
cX24 = c(1,2);
cX42 = c(2,1);

c = cov(x(3,:),x(4,:));
cX34 = c(1,2);
cX43 = c(2,1);

xCov = [cX11,cX12,cX13,cX14;
        cX21,cX22,cX23,cX24;
        cX31,cX32,cX33,cX34;
        cX41,cX42,cX43,cX44;];

end
