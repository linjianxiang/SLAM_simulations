function A_hat = jacobian_robot_motion(X,u,dt)
% X: X = (x,y,theta)
% u: u =(v,w) inputs, velocity and angular velocity 
global L;
x = X(1);
y = X(2);
theta = X(3);
r = u(1,:); 
l = u(2,:);
A_hat = [1 0 -sin(theta +dt*(r-l)/L /4 *(0.5*r+0.5*l));
         0 1 cos(theta +dt*(r-l)/L /4 *(0.5*r+0.5*l));
         0 0 1
        ];

end



function test()
%% the jacobian function generation, which is used for function above
syms x y v w theta dt
states = [x;y;theta];
X = [ x + -v/w * sin(theta) + v/w *sin(theta + w*dt);
 y + v/w * cos(theta)  - v/w *cos(theta + w*dt);
 theta + w*dt];
jacobian(X,states)
end