function [xCov] = getCov3x3(x)
c = cov(x(1,:),x(2,:));
cX11 = c(1,1);
cX12 = c(1,2);
cX21 = c(2,1);
cX22 = c(2,2);

c = cov(x(1,:),x(3,:));
cX13 = c(1,2);
cX31 = c(2,1);
cX33 = c(2,2);

c = cov(x(2,:),x(3,:));
cX23 = c(1,2);
cX32 = c(2,1);

xCov = [cX11,cX12,cX13;
        cX21,cX22,cX23;
        cX31,cX32,cX33];
end