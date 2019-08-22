function [X,P] = EKF_prediction(testing,observation,u,dt)
X = testing.states;
P = testing.P;
states_obsved = observation.states_obsved;
%%%X_r: robot states
%%%P: state error covariance matrix
%%%u: system input
%%%dt: time interval
%%% states_obsved: obsved states indices

r = 1:3;
%X- calculation
X(r) = robot_motion(X(r),u,dt);
X(3) = wrapToPi(X(3));
A_hat_robot = jacobian_robot_motion(X(r),u,dt); 
%P- calcualtion
P(states_obsved,states_obsved) = [A_hat_robot*P(r,r)*A_hat_robot' A_hat_robot*P(r,states_obsved(4:end));
    (A_hat_robot*P(r,states_obsved(4:end)))'  P(states_obsved(4:end),states_obsved(4:end))]; 
end 