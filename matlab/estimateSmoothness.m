function sEst = estimateSmoothness(t,x)
nx = size(x,1);
n = size(x,2);

ts = zeros(1,n-1);
for i = 2:n
    ts(i-1) = t(i) - t(i-1);
end
ts = mean(ts);

sEst = zeros(nx,1);
for i = 1:nx
    func = @(k) findS(k,x(i,:),t);
    % options = optimoptions('particleswarm','SwarmSize',50,...
    %             'HybridFcn',@fmincon,'MaxIterations',4,'Display','iter');
    options = optimoptions('particleswarm','Display','iter',...
                           'MaxIterations',4,'SwarmSize',10,...
                           'HybridFcn',@fmincon);
    sEst(i) = particleswarm(func,1,1e-16,10*ts,options);
end
end