function [X_L,dR,dL] = inverse_landmark_obsv(X_r, L)
%%% X_r: robot states x,y,theta
%%% L:  observed landmark value, [range, angle]
%%% X_L: landmark states: location in the map [x,y]
    x = X_r(1,:);
    y = X_r(2,:);
    theta = X_r(3,:);
    r = L(1,:);
    phi = L(2,:);
    phi0 = wrapToPi(phi+theta);
    x0 = r.*cos(phi0);
    y0 = r.*sin(phi0);
    x_l = x0 + x;
    y_l = y0 + y;
    X_L = [x_l;y_l];
    if nargout >1 
        dR = [1,0,-sin(phi0)*r;
              0,1,cos(phi0)*r];
        dL = [cos(phi0),-sin(phi0)*r;
              sin(phi0),cos(phi0)*r];
    end
    
end

function test()
%%
syms x y theta r phi
states_r = [x;y;theta];
states_l = [r;phi];
X = [ x + r*cos(phi+theta);
      y + r*sin(phi+theta)];
dR = jacobian(X,states_r)
dL = jacobian(X,states_l)

end