function J = jacobian_robot_process_noise(X,u,dt,q)
x = X(1);
y = X(2);
theta = X(3);
v = u(1); 
w = u(2);
n = q.*randn(2,1);
n_v = n(1);
n_w = n(2);

a=(w+n_w); 
b =theta + dt*a;
c = v +n_v;
J = [-sin(theta)/a + sin(b)/a,sin(theta)*c/(a^2)+dt*cos(b)*c/a - sin(b)*c/(a^2);
    cos(theta)/a-cos(b)/a,-cos(theta)*c/(a^2)+ dt*sin(b)*c/a+cos(b)*c/(a^2);
    0,dt];

end

function test()
%% the jacobian function generation, which is used for function above
syms x y v w theta dt n_v n_w n
states = [n_v;n_w];
X = [ x + -(v+n_v)/(w+n_w) * sin(theta) + (v+n_v)/(w+n_w) *sin(theta + (w+n_w)*dt);
 y + (v+n_v)/(w+n_w) * cos(theta)  - (v+n_v)/(w+n_w) *cos(theta + (w+n_w)*dt);
 theta + (w+n_w)*dt];
J =jacobian(X,states)

% X2 = [ x + -(v+n)/(w+n) * sin(theta) + (v+n)/(w+n) *sin(theta + (w+n)*dt);
%  y + (v+n)/(w+n) * cos(theta)  - (v+n)/(w+n) *cos(theta + (w+n)*dt);
%  theta + (w+n)*dt];
% jacobian(X2,n)
end