function PlotLandmarkEllipse(x,P,plt,nSigma)

for i=1:size(x,2)
    r = 3+2*(i-1)+1:3+2*(i-1)+2;
    Pi = P(r,r);
    xi = x(:,i);
    if(~any(diag(Pi)==0))
        [V,D] = eig(Pi);
        y = nSigma*[cos(0:0.1:2*pi);sin(0:0.1:2*pi)];
        el = V*sqrtm(D)*y;
        el = [el el(:,1)]+repmat(xi,1,size(el,2)+1);
        set(plt(i),'xdata',el(1,:),'ydata',el(2,:));
    end
end

