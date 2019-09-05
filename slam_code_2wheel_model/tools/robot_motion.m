% function X_updated = robot_motion(X,u,dt)
% % X: X = (x,y,theta)
% % u: u = v, w
% % dt: time interval
% 
% v = u.v; 
% w = u.w;
% x = X(1);
% y = X(2);
% theta = X(3);
% X_updated = zeros(3,1);
% 
% %robot motion equations
% X_updated(1) = x + -v/w * sin(theta) + v/w *sin(theta + w*dt);
% X_updated(2) = y + v/w * cos(theta)  - v/w *cos(theta + w*dt);
% X_updated(3) = wrapToPi(theta + w*dt);
% 
% end



% multi dim motion
function X_updated = robot_motion(X,u,dt)
% X: X = (x,y,theta)
% u: u = v, w
% dt: time interval
global L;
r = u(1,:); 
l = u(2,:);
[~,m] =size(X);
[~,n] = size(r);
m = max(m,n);
x = X(1,:);
y = X(2,:);
theta = X(3,:);
X_updated = zeros(3,m);


s = (r+l)/2;
dtheta = (r-l)./(2*L);

X_updated(1,:) = x + s.*cos(theta +dtheta/2 *dt);
X_updated(2,:) = y + s.*sin(theta +dtheta/2 *dt);
% X_updated(3,:) = wrapToPi(theta + w*dt);
X_updated(3,:) = theta + dtheta*dt;

end