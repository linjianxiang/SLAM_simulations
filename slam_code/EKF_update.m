function [X,P,L_estimate] = EKF_update(testing,observation,landmark_number)
X = testing.states;
P = testing.P;
Y = observation.Y;
V = observation.V;
landmark_obsved = observation.landmark_obsved;

L_estimate = zeros(2,landmark_number);
r = 1:3;
% nargout 
for i = landmark_obsved
%     L_index = 3+(i-1)*2+1:3+(i-1)*2+2;
%     states_temp = [r,L_index];
%     Y_i = Y(:,i);  %i_th landmark obsv
%     [h,C] = landmark_estimate(X(r),X(L_index)); %landmark estimation and its jacobian;
%     K = P(states_temp,states_temp)*C'*inv(C*P(states_temp,states_temp)*C'+V); %feedback calucaltion
%     E_y = Y_i-h; %output error
%     E_y(2) = wrapToPi(E_y(2));
%     X(states_temp) = X(states_temp) + K*E_y;  % state estimation
%     X(3) = wrapToPi(X(3));
%     P(states_temp,states_temp) = (eye(5)-K*C)*P(states_temp,states_temp); %P update
%     L_estimate(:,i) = X(L_index);
%%%%
    L_index = 3+(i-1)*2+1:3+(i-1)*2+2;
    states_temp = [r,L_index];
    Y_i = Y(:,i);  %i_th landmark obsv
    [h,C_shrink] = landmark_estimate(X(r),X(L_index)); %landmark estimation and its jacobian
    F_x = zeros(5,3+2*landmark_number); % transformation matrix, for less calculation
    F_x(r,r) = eye(3);
    F_x(4:5,L_index) = eye(2);
    C = C_shrink*F_x;
%     K = P*C'*inv(C*P*C'+V); %feedback calucaltion
    K = (P*C')/(C*P*C'+V); %feedback calucaltion
    E_y = Y_i-h; %output error
    E_y(2) = wrapToPi(E_y(2));
    X = X + K*E_y;  % state estimation
    X(3) = wrapToPi(X(3));
    P = (eye(3+2*landmark_number)-K*C)*P; %P update
    L_estimate(:,i) = X(L_index);
end

end