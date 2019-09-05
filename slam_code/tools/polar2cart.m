function c=polar2cart(X,p)
x = X(1,:);
y = X(2,:);
[~,m] = size(p);
theta = X(3,:);
r = p(1,:);
phi = p(2,:);
c = zeros(2,m);
c(1,:) = x + r.*cos(phi+theta);
c(2,:) = y + r.*sin(phi+theta);
end