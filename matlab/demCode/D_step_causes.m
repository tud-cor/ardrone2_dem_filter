%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% D-step of DEM with known input
%
% Function to obtain the filtered state estimates using the D-step of DEM.
% 
% Code source:     https://github.com/ajitham123/DEM_observer
% Original author: Ajith Anil Meera, TU Delft, CoR
% Adjusted by:     Dennis Benders, TU Delft, CoR
% Last modified:   09.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [DEM_t,DEMv_x,Y_embed] = ...
         D_step_causes(A,D_A,B,Da,Bt,Ct,V0y,W0,Y_embed,real_cause,t,...
                       sam_time,nt,nv,ny,p_brain,d_brain)

% Create filter state-space
state_sp_v = ss(Da - Ct'*V0y*Ct - D_A'*W0*D_A, ...
                [Ct'*V0y D_A'*W0*Bt], zeros(1,size(Da,2)), ...
                zeros(1,(p_brain+1)*ny+(d_brain+1)*nv));
state_sp_v = c2d(state_sp_v,sam_time,'zoh');

% Embed the known causes
V_embed = zeros(nv*(d_brain+1),nt);
for i = 1:nt
    V_embed(:,i) = embed_Y(real_cause,d_brain+1,t(i),sam_time);
end
Y_embed(ny*(p_brain+1)+1:end,:) = V_embed;

% Uncomment the following line to set p=0 for outputs only
% Y_embed(ny+1:ny*(p_brain+1),:) = zeros(ny*p_brain,nt);

% Perform state estimation
DEMv_x = zeros(nt,size(state_sp_v.C,2));
for i = 2:nt
    DEMv_x(i,:)=(state_sp_v.A*DEMv_x(i-1,:)' + state_sp_v.B*Y_embed(:,i))';
end

DEM_t = t;

end
