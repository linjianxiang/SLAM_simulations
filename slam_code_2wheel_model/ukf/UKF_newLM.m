function [X, P] = UKF_newLM(robot_pose,testing,observation,L_obsv,sigma_param)
X = testing.states;
P = testing.P;
V = observation.V;
L_index = observation.L_index;
states_obsved = observation.states_obsved;

%convert measurement to states
[measure_sigma,W_c,W_m] = compute_sigma_points(L_obsv,V,sigma_param);
l_sigma = inverse_landmark_obsv(robot_pose, measure_sigma);
X(L_index) = sigma_mean_cal(l_sigma,W_m);
%increase size of error covariance matrix
%new landmark auto covariance
E_l_temp = l_sigma - X(L_index);
P(L_index,L_index) = sigma_cov_cal(E_l_temp,E_l_temp,W_c);
%covariance for new landmark states and other states
[x_sigma_temp,W_c,W_m] = compute_sigma_points(X(states_obsved),P(states_obsved,states_obsved),sigma_param);
x_sigma_temp_ave = sigma_mean_cal(x_sigma_temp,W_m);
x_sigma_temp_ave(3) = wrapToPi(x_sigma_temp_ave(3));
E_x_sigma_temp = x_sigma_temp-x_sigma_temp_ave;
E_x_sigma_temp(3,:) = wrapToPi(E_x_sigma_temp(3,:));
P(states_obsved(1:end-2),L_index) = sigma_cov_cal(E_x_sigma_temp(1:end-2,:),E_x_sigma_temp(end-1:end,:),W_c);
P(L_index,states_obsved(1:end-2)) = P(states_obsved(1:end-2),L_index)'; 
end