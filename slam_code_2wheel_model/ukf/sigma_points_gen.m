function [X_sigma,W_sigma] = sigma_points_gen(X,C,n,k)
%X: states with dim of [m,1] 
%C: error gaussian covaraince
%n: number of sigma points is 2n+1
%k: UKF parameter
%X_sigma: sigma points with dim of [m,2n+1]
%W_sigma: sigma weights with dim of [1,2n+1]

m = length(X);
sigma_size = 2*n+1;
X_sigma = zeros(m,sigma_size);
W_sigma = 1/(2*(n+k))*ones(1,sigma_size);
S = chol(C)';  %cholesky factorization where SS' = C

X_sigma(:,1) = X; %X_sigma0 = mean of X
W_sigma(1) = k/(n+k); % W_sigma0 generate

if n >1
    X_sigma(:,2:n+1) = X+sqrt(n+k)*S(:,1:n);
    X_sigma(:,n+2:end) = X-sqrt(n+k)*S(:,1:n);
elseif n == 1
    X_sigma(:,2) = X+sqrt(n+k)*S(:,1);
    X_sigma(:,3) = X-sqrt(n+k)*S(:,1);
end

end