function plot_results_xh(output,SSE,model,brain,if_UIO,if_cause,xh,t_trim)
%% Plot the results

ax_font_size = 30;
label_font_size = 35;
title_font_size = 40;

%
color = [ 0         0.4470    0.7410;    0.8500    0.3250    0.0980;
          0.9290    0.6940    0.1250;    0.4940    0.1840    0.5560;
          0.4660    0.6740    0.1880;    0.3010    0.7450    0.9330;
          0.6350    0.0780    0.1840];

% Make difference in directly observable states and hidden states
x_obs = 1:brain.nx;
x_obs(xh) = [];

% Define variables for quick plotting
output.DEM_t = output.DEM_t(t_trim);
t = output.DEM_t - output.DEM_t(1);

x = model.ideal_x;

if if_cause
    DEMv_x = output.DEMv_x;
    Y_embed = output.Y_embed;
    kalmv_x = output.kalmfv_x;
else
    DEMx = output.DEM_x;
    kalmx = output.kalman_x;
end

nv = brain.nv;
ny = brain.ny;
nx = brain.nx;
p = brain.p;
d = brain.d;
Da = brain.Da;
At = brain.At;
Bt = brain.Bt;
Ct = brain.Ct;


% Plot state estimates with known input
if if_cause == 1
%     figure('Name','State estimates with known input');
%     subplot(2,1,1);
%     fig1 = plot(t,x(t_trim,x_obs),'Color',color(1,:));
%     hold on;
%     fig2 = plot(t,DEMv_x(t_trim,x_obs),'--','Color',color(2,:));
%     fig3 = plot(t,kalmv_x(x_obs,t_trim),'-.','Color',color(3,:));
%     legend([fig1(1),fig2(1),fig3(1)],...
%            {'Measured observable states','DEM state estimate','KF state estimate'});
%     xlabel('Time (s)');
%     ylabel('State amplitude (-)');
% 
%     subplot(2,1,2);
%     fig1 = plot(t,x(t_trim,xh),'Color',color(1,:));
%     hold on;
%     fig2 = plot(t,DEMv_x(t_trim,xh),'--','Color',color(2,:));
%     fig3 = plot(t,kalmv_x(xh,t_trim),'-.','Color',color(3,:));
%     legend([fig1(1),fig2(1),fig3(1)],...
%            {'Measured hidden state','DEM hidden state estimate','KF hidden state estimate'});
%     xlabel('Time (s)');
%     ylabel('State amplitude (-)');

    figure('Name','Hidden state estimate');
    box on;
    hold on;
    plot(t,x(t_trim,xh),'Color',color(1,:));
    plot(t,DEMv_x(t_trim,xh),'Color',color(2,:));
    plot(t,kalmv_x(xh,t_trim),'Color',color(3,:));
    ax = gca;
    ax.FontSize = ax_font_size;
    legend('Measured',['DEM estimate (SSE = ' num2str(SSE.DEMv.xh) ')'],...
           ['Kalman estimate (SSE = ' num2str(SSE.kalmanv.xh) ')']);
    xlabel('Time (s)','FontSize',label_font_size);
    ylabel('$\dot{\phi}$ (rad/s)','FontSize',label_font_size,...
                                  'Interpreter','latex');
    title('DEM vs Kalman','FontSize',title_font_size);

%     % Plot generalized states
%     figure('Name','Generalized states');
%     for i = 1:p+1
%         subplot(p+1,1,i);
%         plot(t,DEMv_x(t_trim,(i-1)*nx+1:i*nx)');
%     end
% 
%     % Plot generalized outputs
%     figure('Name','Generalized outputs');
%     for i = 1:p+1
%         subplot(p+1,1,i);
%         plot(t,Y_embed((i-1)*ny+1:i*ny,t_trim));
%     end
% 
%     % Plot generalized inputs
%     figure('Name','Generalized inputs');
%     for i = 1:d+1
%         subplot(d+1,1,i);
%         plot(t,Y_embed(ny*(p+1)+(i-1)*nv+1:ny*(p+1)+i*nv,t_trim));
%     end
% 
%     % Plot generalized process noises
%     w_dash = Da*DEMv_x' - At*DEMv_x' - Bt*Y_embed(ny*(p+1)+1:end,:);
%     figure('Name','Generalized process noises');
%     for i = 1:p+1
%         subplot(p+1,1,i);
%         plot(t,w_dash((i-1)*nx+1:i*nx,t_trim));
%     end
% 
%     % Plot generalized measurement noises
%     z_dash = Y_embed(1:ny*(p+1),:) - Ct*DEMv_x';
%     figure('Name','Generalized measurement noises');
%     for i = 1:p+1
%         subplot(p+1,1,i);
%         plot(t,z_dash((i-1)*ny+1:i*ny,t_trim));
%     end




else
    % Plot state estimates with unknown input
    figure('Name','State estimates with unknown input');
    subplot(3,1,1);
    fig1 = plot(model.t,x(:,x_obs),'Color',color(1,:));
    hold on;
    fig2 = plot(t,DEMx(:,x_obs),'--','Color',color(2,:));
    fig3 = plot(model.t,kalmx(x_obs,:)','-.','Color',color(3,:));
%     if if_UIO == 1
%         fig4 = plot(model.t,output.UIO_x_est,'-.','Color',color(4,:));
%         legend([fig1(1),fig2(1),fig3(1),fig4(1)],{'Ideal states (outputs with noise)','DEM states',...
%             'Kalman estimate','UIO estimate'});
%     else
    legend([fig1(1),fig2(1),fig3(1)],{'Measured states (with noise)','DEM state estimate','KF state estimate'});
%     end
    xlabel('Time (s)');
    ylabel('State amplitude (-)');

    subplot(3,1,2);
    fig1 = plot(model.t,x(:,xh),'Color',color(1,:));
    hold on;
    fig2 = plot(t,DEMx(:,xh),'--','Color',color(2,:));
    fig3 = plot(model.t,kalmx(xh,:)','-.','Color',color(3,:));
%     if if_UIO == 1
%         fig4 = plot(model.t,output.UIO_x_est,'-.','Color',color(4,:));
%         legend([fig1(1),fig2(1),fig3(1),fig4(1)],{'Ideal states (outputs with noise)','DEM states',...
%             'Kalman estimate','UIO estimate'});
%     else
    legend([fig1(1),fig2(1),fig3(1)],{'State reference','DEM state estimate','KF state estimate'});
%     end
    xlabel('Time (s)');
    ylabel('State amplitude (-)');

    % Plot input estimates
    subplot(3,1,3);
    plot(model.t,model.real_cause','LineWidth',1);
    hold on;
    plot(t,DEMx(:,brain.nx*(brain.p+1)+1),'--');
%     if if_UIO == 1
%         plot(model.t,output.UIO_v_est,'-.');
%         legend('Ideal causal state','DEM cause estimate','UIO cause estimate');
%     else
    legend('Measured input','DEM input estimate');
%     end
    xlabel('Time (s)');
    ylabel('Input amplitude (-)');
end
end