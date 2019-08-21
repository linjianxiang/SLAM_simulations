function [X,P] = EKF_prediction(X,P,u,dt,states_obseved)
%%%X_r: robot states
%%%P: state error covariance matrix
%%%u: system input
%%%dt: time interval
%%% states_obseved: obsved states indices

r = 1:3;
%X- calculation
X(r) = robot_motion(X(r),u,dt);
X(3) = wrapToPi(X(3));
A_hat_robot = jacobian_robot_motion(X(r),u,dt); 
%P- calcualtion
P(states_obseved,states_obseved) = [A_hat_robot*P(r,r)*A_hat_robot' A_hat_robot*P(r,states_obseved(4:end));
    (A_hat_robot*P(r,states_obseved(4:end)))'  P(states_obseved(4:end),states_obseved(4:end))]; 
end 