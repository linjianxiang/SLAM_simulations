% function [X_sigma,W_c,W_m] = compute_sigma_points(X,C,parameter)
% %X: states with dim of [m,1] 
% %C: error gaussian covaraince
% %n: number of sigma points is 2n+1
% %k: UKF parameter
% %X_sigma: sigma points with dim of [m,2n+1]
% %W_sigma: sigma weights with dim of [1,2n+1]
% alpha = parameter.alpha;
% kappa = parameter.kappa;
% beta = parameter.beta;
% 
% n = length(X);
% sigma_size = 2*n+1;
% lambda = alpha^2 * (n+kappa) - n;
% 
% x_re = repmat(X,1,n);
% W_c = 1/(2*(n+lambda)) *ones(1,sigma_size);
% W_m = 1/(2*(n+lambda)) *ones(1,sigma_size);
% S = chol(C)';  %cholesky factorization where SS' = C
% S = sqrt(n+lambda) * S
% 
% W_m(1) = lambda / (n+lambda); % W_m0 generate
% W_c(1) = W_m(1) + (1 -alpha^2 + beta); % W_c0 generate
% 
% X_sigma = [X, x_re + S, x_re - S];
% 
% end

function [X_sigma,W_c,W_m] = compute_sigma_points(X,C,parameter,n_give)
% Computes the 2n+1 sigma points according to the unscented transform,
% where n is the dimensionality of the mean vector X.
% The sigma points should form the columns of sigma_points,
% i.e. sigma_points is an nx2n+1 matrix.

n = length(X); % state dimension
alpha = parameter.alpha;
kappa = parameter.kappa;
beta = parameter.beta;
if nargin == 4
   n = n_give ;
end

% lambda = max(alpha^2 * (n + kappa) - n, 1);
lambda = alpha^2 * (n + kappa) - n;
scale = lambda + n;
[U, D, V] = svd(C); %[V,D] = eig(sigma);
D = sqrt(D);
% if n >3
%     D(4:end,4:end) = min(D(4:end,4:end), 0.01 * eye(n-3)); % lower bound explained below
% end
sigmasqr = ...sqrt(n+lambda) * Seems to be too big!
           ... sqrtm(sigma); %chol(sigma);
           U * D * V'; %V * D / V;
if nargin == 4
   sigmasqr = sigmasqr(1:length(X),:) ;
end
Xreplicated = repmat(X, 1, n);
X_sigma = [X, Xreplicated + sigmasqr, Xreplicated - sigmasqr];

% Computing the weights for recovering the mean and variance.
% Note: weights are column vectors, to allow vector Xltiply w/ samples
W_m = [lambda/scale; repmat(1/(2*scale), 2*n, 1)]';
W_c = W_m;
W_c(1) = W_c(1) + 1 - alpha^2 + beta;
end