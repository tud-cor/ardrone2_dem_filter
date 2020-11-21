function [xCov] = getCov7x7(x)
% Covariances related to x1
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

c = cov(x(1,:),x(5,:));
cX15 = c(1,2);
cX51 = c(2,1);
cX55 = c(2,2);

c = cov(x(1,:),x(6,:));
cX16 = c(1,2);
cX61 = c(2,1);
cX66 = c(2,2);

c = cov(x(1,:),x(7,:));
cX17 = c(1,2);
cX71 = c(2,1);
cX77 = c(2,2);


% Covariances related to x2
c = cov(x(2,:),x(3,:));
cX23 = c(1,2);
cX32 = c(2,1);

c = cov(x(2,:),x(4,:));
cX24 = c(1,2);
cX42 = c(2,1);

c = cov(x(2,:),x(5,:));
cX25 = c(1,2);
cX52 = c(2,1);

c = cov(x(2,:),x(6,:));
cX26 = c(1,2);
cX62 = c(2,1);

c = cov(x(2,:),x(7,:));
cX27 = c(1,2);
cX72 = c(2,1);


% Covariances related to x3
c = cov(x(3,:),x(4,:));
cX34 = c(1,2);
cX43 = c(2,1);

c = cov(x(3,:),x(5,:));
cX35 = c(1,2);
cX53 = c(2,1);

c = cov(x(3,:),x(6,:));
cX36 = c(1,2);
cX63 = c(2,1);

c = cov(x(3,:),x(7,:));
cX37 = c(1,2);
cX73 = c(2,1);


% Covariances related to x4
c = cov(x(4,:),x(5,:));
cX45 = c(1,2);
cX54 = c(2,1);

c = cov(x(4,:),x(6,:));
cX46 = c(1,2);
cX64 = c(2,1);

c = cov(x(4,:),x(7,:));
cX47 = c(1,2);
cX74 = c(2,1);


% Covariances related to x5
c = cov(x(5,:),x(6,:));
cX56 = c(1,2);
cX65 = c(2,1);

c = cov(x(5,:),x(7,:));
cX57 = c(1,2);
cX75 = c(2,1);


% Covariances related to x6
c = cov(x(6,:),x(7,:));
cX67 = c(1,2);
cX76 = c(2,1);


% Complete covariance matrix
xCov = [cX11,cX12,cX13,cX14,cX15,cX16,cX17;
        cX21,cX22,cX23,cX24,cX25,cX26,cX27;
        cX31,cX32,cX33,cX34,cX35,cX36,cX37;
        cX41,cX42,cX43,cX44,cX45,cX46,cX47;
        cX51,cX52,cX53,cX54,cX55,cX56,cX57;
        cX61,cX62,cX63,cX64,cX65,cX66,cX67;
        cX71,cX72,cX73,cX74,cX75,cX76,cX77];
end