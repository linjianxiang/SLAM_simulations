function P = sigma_cov_cal(E_sigma1,E_sigma2,W_c)
%Z_sigma, the error of substracting x by X
    [n,m] = size(E_sigma1);
    [n2,~] = size(E_sigma2);
    P = zeros(n,n2);
    for  i =1:m
       P = P + W_c(i) *  E_sigma1(:,i)*E_sigma2(:,i)';
    end
    
end