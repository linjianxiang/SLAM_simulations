function m = landmarks_obsv(X,L)
% X: X = (x,y,theta)
% L: L = [L1_x Ln_y] landmark coordinate
% noise_info: struct('variance',noise_variance,'ratio',noise_ratio);

x = X(1,:);
y = X(2,:);
theta = X(3,:);
L_x = L(1,:);
L_y = L(2,:);
%landmarks observation, measurment noise added

r = sqrt((L_x - x).^2 + (L_y - y).^2);
phi = atan2((L_y-y),(L_x-x))-theta;   
phi = wrapToPi(phi);
m = [r;phi];

end