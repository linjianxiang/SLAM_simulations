function E = landmark_sum_mse(X1,X2)
    e = X1 - X2;
    E = sum(e.^2,2);
end