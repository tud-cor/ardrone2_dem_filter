function [Sigma,s] = estimateNoiseCharacteristics(t,x,plotAc,plotSMSE)
% Get noise dimensions
nx = size(x,1);
n  = size(x,2);

% Calculate sampling time/frequency
ts = zeros(1,n-1);
for i = 1:n-1
    ts(i) = t(i+1) - t(i);
end
ts = mean(ts);

% Zero the data if necessary
% for i = 1:nx
%     if mean(x(i,:)) ~= 0
%         x(i,:) = x(i,:) - mean(x(i,:));
%     end
% end


% Calculate covariance matrix Sigma
% (assuming independent measurement signals)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mu       = mean(x,2);
sigma    = std(x,1,2);
Sigma    = diag(sigma);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Calculate kernel width of Gaussian filter that is assumed to produce
% coloured noise from white noise
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize dimensions
nLags    = 100;
nStdFilt = 1;
sMin     = 1e-16;
sStep    = 1e-4;
sMax     = 10*ts;
acX      = zeros(nx,nLags+1);
lagsX    = zeros(nx,nLags+1);
boundsX  = zeros(nx,2);

% Compute autocorrelation of z
for i = 1:nx
    [acX(i,:),lagsX(i,:),boundsX(i,:)] = ...
        autocorr(x(i,:),'NumLags',nLags,'NumSTD',1);

    if plotAc
        figure('Name',['Autocorrelation of x' num2str(i)]);
        stem(lagsX(i,:),acX(i,:),'filled');
        hold on
        plot(lagsX(i,:),boundsX(i,:)'*ones(size(lagsX(i,:))),'r');
        xlabel('Lag');
        ylabel('Autocorrelation value');
        legend('Autocorrelation','Confidence interval');
    end
end


% Compute autocorrelation of white noise signal filtered with a Gaussian
% filter with different kernel widths

% Determine amount of lags to fit autocorrelation function on:
% only take the lags into account until the autocorrelation goes below 0,
% or goes up again
nLagsFit = zeros(nx,1);
for i = 1:nx
    for j = 2:nLags+1
        if acX(i,j) - acX(i,j-1) > 0 || acX(i,j) < 0
            break;
        end
    end
    nLagsFit(i) = j-1;
end

% Select random number generation (rng)
rng(1);

% Create time sequence for Gaussian filter
tau = linspace(-t(end),t(end),2*n-1);

% Initialize coloured noise signal properties
omegaRef = zeros(nx,n);
for i = 1:nx
    omegaRef(i,:) = normrnd(mu(i),sigma(i),[1,n]);
end
mseResult.sRef = sMin:sStep:sMax;
mseResult.mseRef = zeros(nx,length(mseResult.sRef));

% Create coloured noise signal by convoluting a white noise signal with a
% Gaussian filter
for i = 1:length(mseResult.sRef)
    s = mseResult.sRef(i);
    h = sqrt(1/ts*s*sqrt(pi))*exp(-tau.^2/(2*s^2));
    ref = zeros(nx,n);
    acRef = zeros(nx,nLags+1);
    lagsRef = zeros(nx,nLags+1);
    boundsRef = zeros(nx,2);

    for j = 1:nx
        ref(j,:) = conv(h,omegaRef(j,:),'valid');
        [acRef(j,:),lagsRef(j,:),boundsRef(j,:)] = ...
            autocorr(ref(j,:),'NumLags',nLags,'NumSTD',nStdFilt);
        mseResult.mseRef(j,i) = ...
            mean((acX(j,1:nLagsFit(j))-acRef(j,1:nLagsFit(j))).^2);
    end
end

% Plot estimation of kernel width
if plotSMSE
    for i = 1:nx
        figure('Name',['Estimated kernel width for x' num2str(i), ...
                       ' using LS']);
        plot(mseResult.sRef,mseResult.mseRef(i,:),'-o');
        hold on;
        [~,idx] = min(mseResult.mseRef(i,:));
        xline(mseResult.sRef(idx),'Color',[0 0.4470 0.7410]);
        xlabel('s (s)');
        ylabel('MSE value');
        legend(['MSE for s in range ' num2str(sMin) ':' num2str(sStep) ...
                ':' num2str(sMax) ':'],'Estimated s');
    end
end

% Calculate smoothness/kernel width of every input signal
s    = zeros(nx,1);
sMSE = zeros(nx,1);
for i = 1:nx
    [sMSE(i),idx] = min(mseResult.mseRef(i,:));
    s(i) = mseResult.sRef(idx);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end