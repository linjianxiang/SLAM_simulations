function m = sigma_mean_cal(sigma_points,weights)
%sigma_points: dim of [n,2n+1]  where 2n+1 is number of sigma points
%weights: dim of [1,2n+1]
%m:mean of sigma points  dim of [n,1]

m = sum(sigma_points.*weights,2);

end