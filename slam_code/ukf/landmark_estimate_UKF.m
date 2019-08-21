function h = landmark_estimate_UKF(X,L)
% X: X = (x,y,theta) robot states
% L: L = [Li_x Li_y]' landmark states
% h: h = [r phi]' estimated observation
% C: jacobian matrix of h
x = X(1,:);
y = X(2,:);
theta = X(3,:);
L_x = L(1,:);
L_y = L(2,:);
h = [sqrt((L_x-x).^2 +(L_y-y).^2);
      wrapToPi(atan2((L_y-y),(L_x-x))-theta)
    ];

end