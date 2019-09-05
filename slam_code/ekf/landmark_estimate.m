function [h,C] = landmark_estimate(X,L)
% X: X = (x,y,theta) robot states
% L: L = [Li_x Li_y]' landmark states
% h: h = [r phi]' estimated observation
% C: jacobian matrix of h
x = X(1);
y = X(2);
theta = X(3);
L_x = L(1);
L_y = L(2);
h = [sqrt((L_x-x)^2 +(L_y-y)^2);
     wrapToPi(atan2((L_y-y),(L_x-x))-theta)
    ];
%for simplicity introduce R (radius) S (sin(phi0))
R = sqrt((L_x-x)^2 +(L_y-y)^2);
S = (L_y-y)/(L_x-x);
% C = [(x-L_x)/R      (y-L_y)/R       0       (L_x-x)/R       (L_y-y)/R;
%      (L_y-y)/(R^2)  (x-L_x)/(R^2)   -1      (L_y-y)/(R^2)   (L_x-x)/(R^2)];
C = [(x-L_x)/R      (y-L_y)/R       0       (L_x-x)/R       (L_y-y)/R;
     (L_y-y)/(R^2)  -1/((L_x-x)*(S^2+1))   -1      -(L_y-y)/(R^2)   1/((L_x-x)*(S^2+1))];
end


function testing()
%% for testing the correctness of the jacobian matrix for above function
syms x y L_x L_y theta
states = [x;y;theta;L_x;L_y];
h = [sqrt((L_x-x)^2 +(L_y-y)^2);
     atan((L_y-y)/(L_x-x))-theta
    ];
J = jacobian(h,states)
end