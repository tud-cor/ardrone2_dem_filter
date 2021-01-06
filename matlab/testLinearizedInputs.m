load tempUEs.mat t xLin B0 uLin w0;
BEs = B0;
uLinEs = uLin;
wEs = w0;
load tempUOwnWork.mat B0 uLin0 w0;
BO = B0;
uLinO = uLin0;
wO = w0;

figure;
plot(t,BEs*uLinEs);
hold on;
plot(t,BO*uLinO);

figure;
plot(t(1:end-1),wEs);
hold on;
plot(t(1:end-1),wO);