%% Initialize MATLAB interface
clear;
close all;
clc;


%% Create finite differences matrix
ts = 0.04;
p = 3;
o = 6;
n = 1;
method = 'c';
E = f_finitediffmat(ts,p,o,n,method);

nOffset = 0.5*(p+o-1);
tOffset = nOffset*ts;


%% Create test signal
tSim = 5;
a = 1;
f = 0.2;

n = tSim/ts+1;
omega = 2*pi*f;

t = linspace(-tOffset,tSim+tOffset,n+2*nOffset)';
nTot = length(t);

w = normrnd(0,0.1,[nTot,1]);

y = a*sin(omega*t) + w;


%% Calculate exact derivatives
y1E = a*omega*cos(omega*t);
y2E = -a*omega^2*sin(omega*t);
y3E = -a*omega^3*cos(omega*t);


%% Calculate simple derivative
y1Sb = zeros(n,1); % Derivative using previous and current sample
y1Sa = zeros(n,1); % Derivative using current and next sample
y1Sm = zeros(n,1); % Mean of both simple derivative calculations
for i = 1:n
    y1Sb(i) = (y(nOffset+i)-y(nOffset+i-1))/ts;
    y1Sa(i) = (y(nOffset+i+1)-y(nOffset+i))/ts;
end
y1Sm = (y1Sb+y1Sa)/2;


%% Calculate derivatives using finite differences approach
blockYF = p+1;
blockY = 2*nOffset+1;
yF = zeros(blockYF*n,1);
for i = 1:n
    yF((i-1)*blockYF+1:i*blockYF) = E*y(i:i+blockY-1);
end

yFR = zeros(n,1);
y1F = zeros(n,1);
y2F = zeros(n,1);
y3F = zeros(n,1);
for i = 1:n
    yFR(i) = yF((i-1)*blockYF+1);
    y1F(i) = yF((i-1)*blockYF+2);
    y2F(i) = yF((i-1)*blockYF+3);
    y3F(i) = yF((i-1)*blockYF+4);
end


%% Plot result
tFD = t(nOffset+1:end-nOffset);
legendDer = cell(2,1);
legendDer{1} = 'Exact';
legendDer{2} = 'Finite differences';

figure('Name','Test signal');
plot(t,y,'-o');
hold on;
plot(tFD,yFR);

figure('Name','First-order derivative');
plot(t,y1E,'-o');
hold on;
% plot(tFD,y1Sb,'-o');
% plot(tFD,y1Sa,'-o');
plot(tFD,y1Sm,'-o');
plot(tFD,y1F,'-o');
legend('Exact','Simple before','Simple after','Simple mean',...
       'Finite differences');

figure('Name','Second-order derivative');
plot(t,y2E);
hold on;
plot(tFD,y2F);
legend(legendDer);

figure('Name','Third-order derivative');
plot(t,y3E);
hold on;
plot(tFD,y3F);
legend(legendDer);
