function err = getErr(x,ref,method)
% The 2-norm (||x||_2) is used as error metric.

% Check inputs
[nRX,nCX] = size(x);
[nRR,nCR] = size(ref);

% Assume that the amount of samples is larger than the amount of state and
% ensure that the columns represent the states
if nRX > nCX
    x = x';
    nRX = nCX;
    nCX = nRX;
end
if nRR > nCR
    ref = ref';
    nRR = nCR;
    nCR = nRR;
end

% Give errors and warnings if needed
if nRX ~= nRR
    error(['x and ref have a different amount of states. Error cannot '...
           'be calculated.']);
end
if nCX ~= nCR
    error(['x and ref have a different amount of samples. Error cannot '...
           'be calculated']);
end
if nRX < 10 && nCX < 10
    warning(['Amount of time samples is below 10. Error may not be '...
             'accurate']);
end

% Calculate Mean Squared Error (MSE)
se = zeros(1,nCX);
for i = 1:nCX
    se(i) = norm(ref(:,i) - x(:,i))^2;
end
mse = mean(se);

% Provide the error as indicated by method
% MSE
if strcmp(method,'MSE') || strcmp(method,'Mse') || strcmp(method,'mse')
    err = mse;

% Variance Accounted For (VAF)
elseif strcmp(method,'VAF') || strcmp(method,'Vaf') || strcmp(method,'vaf')
    % Calculate Mean Squared Norm (MSN) of ref
    snRef = zeros(1,nCR);
    for i = 1:nCR
        snRef(i) = norm(ref(:,i))^2;
    end
    msnRef = mean(snRef);

    % Calculate VAF using MSE and MSN
    err = max(0,...
             (1 - mse/msnRef)*100);
end
end