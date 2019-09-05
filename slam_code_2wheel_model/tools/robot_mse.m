function E = robot_mse(X1,X2)
    e = X1-X2;
    E(3,:) = rad2deg(wrapToPi(e(3)));
    E(1:2,:) = e(1:2).^2;
end