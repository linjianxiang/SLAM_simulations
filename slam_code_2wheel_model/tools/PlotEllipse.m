function PlotEllipse(x,P,plt,nSigma)
P = P(1:2,1:2); % we are only interested in x and y part
x = x(1:2);
if(~any(diag(P)==0))
    [V,D] = eig(P);
    y = nSigma*[cos(0:0.1:2*pi);sin(0:0.1:2*pi)];
    el = V*sqrtm(D)*y;
    el = [el el(:,1)]+repmat(x,1,size(el,2)+1);
    set(plt,'xdata',el(1,:),'ydata',el(2,:));

end