function [X,P] = UKF_prediction_v2(X,P,u,Q,dt,sigma_param,states_obsved,RL_cov)
%%%X_r: robot states
%%%P: state error covariance matrix
%%%u: system input
%%%dt: time interval
%%% states_obseved: obsved states indices

r = 1:3;
l_states = states_obsved(4:end);
%state predict
Q1 = zeros(5,5);
Q1(r,r) =P(r,r);
Q1(4:5,4:5) = Q;
X_temp = zeros(5,1);
X_temp(r) = X(r);
X_temp(4:5) = u;
[X_sigma_temp,W_c,W_m] = compute_sigma_points(X_temp,Q1,sigma_param);
X_sigma_r = X_sigma_temp(r,:);
u_sigma_r = X_sigma_temp(4:5,:);
X_Robot_sigma = robot_motion(X_sigma_r,u_sigma_r,dt);  %sigma point after motion
X_Robot_mean = sigma_mean_cal(X_Robot_sigma,W_m); %state mean
X_Robot_mean(3) = wrapToPi(X_Robot_mean(3));
E_x_robot = X_Robot_sigma - X_Robot_mean;
E_x_robot(3,:) = wrapToPi(E_x_robot(3,:));

P(r,r) = sigma_cov_cal(E_x_robot,E_x_robot,W_c);%+sigma_cov_cal(E_x_robot2,E_x_robot2,W_c);
if ~RL_cov
    P(r,l_states) = zeros(3,length(l_states));
    P(l_states,r) = zeros(3,length(l_states))';
else
    [X_sigma_temp,W_c,W_m] = compute_sigma_points(X(states_obsved),P(states_obsved,states_obsved),sigma_param);
    X_mean = sigma_mean_cal(X_sigma_temp,W_m);
    E_X = X_sigma_temp - X_mean;
    P(r,l_states) = sigma_cov_cal(E_X(r,:),E_X(4:end,:),W_c);
    P(l_states,r) = P(r,l_states)';
end
    
X(r) = X_Robot_mean;
end 