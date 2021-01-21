%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Noise properties estimation
%
% Function to estimate the noise parameters: covariance matrix and
% smoothness.
% 
% Author:        Dennis Benders, TU Delft, CoR
% Last modified: 20.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Sigma,s] = estimateNoiseCharacteristics(t,x,plotAc,plotSSSE)

% Define font sizes
axFontSize = 23;
labelFontSize = 35;
titleFontSize = 40;

% Get noise dimensions
nx = size(x,1);
n  = size(x,2);

% Calculate sampling time/frequency
ts = zeros(1,n-1);
for i = 1:n-1
    ts(i) = t(i+1) - t(i);
end
ts = mean(ts);

% Zero the data
for i = 1:nx
    if mean(x(i,:)) ~= 0
        x(i,:) = x(i,:) - mean(x(i,:));
    end
end


% Calculate covariance matrix Sigma
% (assuming independent measurement signals only for nx >= 5)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mu = mean(x,2);
if nx == 1
    Sigma = std(x);
elseif nx == 2
    Sigma = cov(x(1,:),x(2,:));
elseif nx == 3
    Sigma = getCov3x3(x);
elseif nx == 4
    Sigma = getCov4x4(x);
else
    sigma = std(x,0,2);
    Sigma = diag(sigma);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Calculate kernel width of Gaussian filter that is assumed to produce
% coloured noise from white noise (noise smoothness)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize dimensions
nLags    = 60;
sMin     = 1e-16;
sStep    = 1e-4;
sMax     = 0.25;
acX      = zeros(nx,nLags+1);
lagsX    = zeros(nx,nLags+1);
boundsX  = zeros(nx,2);

% Compute autocorrelation of x
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


% Compute autocorrelation of a Gaussian filter and fit on autocorrelation
% of x

% Determine amount of lags to fit autocorrelation function on
nLagsFit = zeros(nx,1);
fprintf(['Fill in the amount of samples to take into account when ',...
         'estimating the noise smoothness (e.g. "nLagsFit = 18" or ',...
         '"nLagsFit(1) = 18" and "nLagsFit(2) = 20") and press enter ',...
         'and F5 to continue.\n']);
keyboard;

% Close all figures drawn until now
close all;

% Create time sequence for Gaussian filter
tau = linspace(-t(end),t(end),2*n-1);

% Calculate SSE for a range of Gaussian filter kernel widths
sseResult.sRef = sMin:sStep:sMax;
sseResult.sseRef = zeros(nx,length(sseResult.sRef));
for i = 1:length(sseResult.sRef)
    s = sseResult.sRef(i);
    hA = exp(-tau.^2/(4*s^2));

    for j = 1:nx
        sseResult.sseRef(j,i) = ...
            sum((acX(j,1:nLagsFit(j))-hA(n:n+nLagsFit(j)-1)).^2);
    end
end

% Calculate and plot smoothness/kernel width of every input signal, based
% on minimal SSE
s    = zeros(nx,1);
sMSE = zeros(nx,1);
for i = 1:nx
    [sMSE(i),idx] = min(sseResult.sseRef(i,:));
    s(i) = sseResult.sRef(idx);

    if plotSSSE
        figure('Name',['Estimated kernel width for x' num2str(i), ...
                       ' using LS']);
        plot(sseResult.sRef,sseResult.sseRef(i,:),'-o');
        box on;
        hold on;
        xline(s(i),'label',['s = ' num2str(s(i))],...
              'Color',[0.8500 0.3250 0.0980],'FontSize',axFontSize,...
              'LineWidth',3);
        legend(['SSE for s in range ' num2str(sMin) ':' num2str(sStep) ...
                ':' num2str(sMax)],['Estimated s: ' num2str(s(i))],...
                'Location','southeast');
        xlabel('s (s)','FontSize',labelFontSize);
        ylabel('SSE','FontSize',labelFontSize);
        title('SSE for different smoothness values');
        ax = gca;
        ax.FontSize = axFontSize;
    end
end

% Plot autocorrelation with fitted autocorrelation of Gaussian filter
% Create autocorrelation of Gaussian filter
lags = linspace(1,2*n-1,2*n-1) - n;
lags = lags(n-nLags:n+nLags);
for i = 1:nx
    h = exp(-tau.^2/(4*s(i)^2));
    h = h(n-nLags:n+nLags);
    figure('Name',['Autocorrelation of x' num2str(i) ' and fitted '...
                   'autocorrelation of Gaussian filter']);
    stem([-lagsX(i,2:end),lagsX(i,:)],[acX(i,2:end),acX(i,:)],'filled');
    box on;
    hold on;
    plot(lags,h,'LineWidth',3);
    legend('Autocorrelation','Fitted autocorrelation of Gaussian filter');
    xlabel('Number of lags','FontSize',labelFontSize);
    ylabel('Autocorrelation','FontSize',labelFontSize);
    title(['Gaussian filter autocorrelation fitted on autocorrelation ',...
           'of coloured noise'],...
          'FontSize',titleFontSize);
    ax = gca;
    ax.FontSize = axFontSize;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
