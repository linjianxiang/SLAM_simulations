function [X,P,L_estimate] = UKF_update_testing(testing,observation,landmark_number,sigma_param)
X = testing.states;
P = testing.P;
Y = observation.Y;
V = observation.V;
landmark_obsved = observation.landmark_obsved;
states_obsved = observation.states_obsved;
L_estimate = zeros(2,landmark_number);
Y_v = zeros(2*landmark_number,1);
v = [V(1,1),V(2,2)]';
r = 1:3;
v_obsved =[];

for i = 1:length(landmark_obsved)
   v_obsved = [v_obsved;v];
   Y_v(2*(i-1)+1:2*(i-1)+2) = Y(i);
end
V_obsved = diag(v_obsved);
% 
% [X_sigma,W_c,W_m] = compute_sigma_points(X(states_obsved),P(states_obsved,states_obsved),sigma_param);
% X_sigma(3,:) = wrapToPi(X_sigma(3,:));
% Z = zeros(length(landmark_obsved)*2,1);
% Z_sigma = zeros(2,length(states_obsved)*2+1);
% for i = 1:length(landmark_obsved)
%     L_index = 3+2*(i-1)+1:3+2*(i-1)+2; 
%     zi_sigma = landmark_estimate_UKF(X_sigma(r,:),X_sigma(L_index,:)); % approach one
%     z_temp = sigma_mean_cal(zi_sigma,W_m);
%     z_temp(2) = wrapToPi(z_temp(2));
%     Z(L_index-3) = z_temp;
%     Z_sigma(L_index-3,:) = zi_sigma;      
% end
% E_x = X_sigma - X(states_obsved);
% E_x(3,:) = wrapToPi(E_x(3,:));
% E_z = Z_sigma - Z;
% E_z(2:2:end,:) = wrapToPi(E_z(2:2:end,:));
% P_z = sigma_cov_cal(E_z,E_z,W_c);
% P_z = P_z + V_obsved;
% P_xy = sigma_cov_cal(E_x,E_z,W_c);
% K = P_xy * inv(P_z);
% D = Y_v(states_obsved(4:end)-3) - Z;
% D(2:2:end) = wrapToPi(D(2:2:end));
% X_new = X(states_obsved) + K* D;
% if abs(X_new(1)) > abs(3*X(1))
%    a = 1; 
% end
% X(states_obsved) = X_new;
% X(3) = wrapToPi(X(3));
% P(states_obsved,states_obsved) = P(states_obsved,states_obsved) - K*P_z*K';
% for i = 1:landmark_number
%     L_index = 3+2*(i-1)+1:3+2*(i-1)+2;
%     L_estimate(:,i) = X(L_index);
% end

    [X_sigma,W_c,W_m] = compute_sigma_points(X(states_obsved),P(states_obsved,states_obsved),sigma_param);
    X_sigma(3,:) = wrapToPi(X_sigma(3,:));
    Z = zeros(length(landmark_obsved)*2,1);
    Z_sigma = zeros(2,length(states_obsved)*2+1);
    for i = 1:length(landmark_obsved)
        L_index = 3+2*(i-1)+1:3+2*(i-1)+2; 
        zi_sigma = landmark_estimate_UKF(X_sigma(r,:),X_sigma(L_index,:)); % approach one
        z_temp = sigma_mean_cal(zi_sigma,W_m);
        z_temp(2) = wrapToPi(z_temp(2));
        Z(L_index-3) = z_temp;
        Z_sigma(L_index-3,:) = zi_sigma;      
    end
    E_x = X_sigma - X(states_obsved);
    E_x(3,:) = wrapToPi(E_x(3,:));
    E_z = Z_sigma - Z;
    E_z(2:2:end,:) = wrapToPi(E_z(2:2:end,:));
    P_z = sigma_cov_cal(E_z,E_z,W_c);
    P_z = P_z + V_obsved;
    P_xy = sigma_cov_cal(E_x,E_z,W_c);
    K = P_xy/(P_z);
    D = Y_v(states_obsved(4:end)-3) - Z;
    D(2:2:end) = wrapToPi(D(2:2:end));
    X_new = X(states_obsved) + K* D;
    X(states_obsved) = X_new;
    X(3) = wrapToPi(X(3));
    P(states_obsved,states_obsved) = P(states_obsved,states_obsved) - K*P_z*K';
    for i = 1:landmark_number
        L_index = 3+2*(i-1)+1:3+2*(i-1)+2;
        L_estimate(:,i) = X(L_index);
    end
end