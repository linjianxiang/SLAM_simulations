function A_hat = jacobian_robot_motion(X,u,dt)
% X: X = (x,y,theta)
% u: u =(v,w) inputs, velocity and angular velocity 
x = X(1);
y = X(2);
theta = X(3);
v = u(1,:); 
w = u(1,:);
A_hat = [1 0 v/w*(-cos(theta) + cos(dt*w+theta));
         0 1 v/w*(-sin(theta) + sin(dt*w+theta));
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