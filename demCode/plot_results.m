function plot_results(output,model,brain,if_UIO,if_cause)
%% Plot the results

%
color = [ 0         0.4470    0.7410;    0.8500    0.3250    0.0980;
          0.9290    0.6940    0.1250;    0.4940    0.1840    0.5560;
          0.4660    0.6740    0.1880;    0.3010    0.7450    0.9330;
          0.6350    0.0780    0.1840];

% Plot state estimates with known input
if if_cause == 1
    figure('Name','State estimates with known input');
    fig1 = plot(output.DEM_t,model.ideal_x,'Color',color(1,:));
    hold on;
    fig2 = plot(output.DEM_t,output.DEMv_x(:,1:brain.nx),'--','Color',color(2,:));
    fig3 = plot(output.DEM_t,output.kalmfv_x,'-.','Color',color(3,:));
    legend([fig1(1),fig2(1),fig3(1)],...
           {'Measured states (with noise)','DEM state estimate','KF state estimate'});
    xlabel('Time (s)');
    ylabel('State amplitude (-)');
end
% Plot state estimates with unknown input
figure('Name','State estimates with unknown input');
subplot(2,1,1);
fig1 = plot(model.t,model.ideal_x,'Color',color(1,:));
hold on;
fig2 = plot(output.DEM_t,output.DEM_x(:,1:brain.nx),'--','Color',color(2,:));
fig3 = plot(model.t,output.kalman_x','-.','Color',color(3,:));
%     if if_UIO == 1
%         fig4 = plot(model.t,output.UIO_x_est,'-.','Color',color(4,:));
%         legend([fig1(1),fig2(1),fig3(1),fig4(1)],{'Ideal states (outputs with noise)','DEM states',...
%             'Kalman estimate','UIO estimate'});
%     else
legend([fig1(1),fig2(1),fig3(1)],{'Measured states (with noise)','DEM state estimate','KF state estimate'});
%     end
xlabel('Time (s)');
ylabel('State amplitude (-)');

% Plot input estimates
subplot(2,1,2);
fig1 = plot(model.t,model.real_cause','Color',color(1,:),'LineWidth',1);
hold on;
fig2 = plot(output.DEM_t,output.DEM_x(:,brain.nx*(brain.p+1)+1:brain.nx*(brain.p+1)+brain.nv),'--','Color',color(2,:));
%     if if_UIO == 1
%         plot(model.t,output.UIO_v_est,'-.');
%         legend('Ideal causal state','DEM cause estimate','UIO cause estimate');
%     else
legend([fig1(1),fig2(1)],'Measured input','DEM input estimate');
%     end
xlabel('Time (s)');
ylabel('Input amplitude (-)');
end