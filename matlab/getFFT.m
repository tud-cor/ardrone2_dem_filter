function [f,pX1] = getFFT(t,x)
n = length(t);

ts = zeros(1,n-1);
for i = 1:n-1
    ts(i) = t(i+1) - t(i);
end
ts = mean(ts);
fs = 1/ts;
f = fs*(0:(n/2))/n;

fX = fft(x,n,2);
pX2 = abs(fX/n);
pX1 = pX2(:,1:n/2+1);
pX1(:,2:end-1) = 2*pX1(:,2:end-1);
end