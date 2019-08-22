function [X,P] = EKF_newLM(robot_pose,testing,observation,L_obsv)
X = testing.states;
P = testing.P;
V = observation.V;
states_obsved = observation.states_obsved;
L_index = observation.L_index;
%%% robot_pose: real robot location
%%% X: all states
%%% P:state error cov
%%% L_obsv: new landmark observation
%%% states_obsved: observed state indices, include robot states and observed landmark states
%%% L_index: new landmark index
    r = 1:3;
    %convert measurement to states, new landmark states
    [X(L_index),dR,dL] = inverse_landmark_obsv(robot_pose, L_obsv);
    %increase size of error covariance matrix
    P(L_index,L_index) = dR * P(r,r) *dR' + dL * V *dL';
    P(L_index,states_obsved(1:end-2)) = dR*P(r,states_obsved(1:end-2));
    P(states_obsved(1:end-2),L_index) =P(L_index,states_obsved(1:end-2))';
end