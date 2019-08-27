function dx = LS_optimization(constraints,odom)

len = size(odom,2);
needToAddPrior = true;
H = zeros(3*len, 3*len);
b = zeros(3*len, 1);
for i = 1:(size(constraints.pose_pose_edges,1))
   
    %constrains two side index
    index = constraints.pose_pose_edges(i,:);
    i1 = index(1);
    i2 = index(2);
%     r1 = ((3*i+1):(3*(i+1)))-3;
%     r2 = r1+3;
    r1 = ((3*i1+1):(3*(i1+1)))-3;
    r2 = ((3*i2+1):(3*(i2+1)))-3;
    
    x1 = v2t(odom(:,i1));
    x2 = v2t(odom(:,i2));
 
    [e, A, B] = linearize_pose_pose_constraint(x1, x2, constraints.z(i,:));
    % TODO: compute and add the term to H and b
    %Phi = edge.information;

    Phi = eye(3);
    H(r1,r1) = H(r1,r1)+ A'*Phi*A;
    H(r2,r2) = H(r2,r2)+B'*Phi*B; 
    H(r2,r1) = H(r2,r1)+B'*Phi*A;
    H(r1,r2) = H(r1,r2)+A'*Phi*B;
    
    b(r1) =b(r1)+ (e'*Phi*A)';
    b(r2) = b(r2) + (e'*Phi*B)';

    if (needToAddPrior)
      % TODO: add the prior for one pose of this edge
      % This fixes one node to remain at its current location
      H(1,1) = H(1,1) + 1;
      H(2,2) = H(2,2) + 1;
      H(3,3) = H(3,3) + 1;
      needToAddPrior = false;
    end
end
dx = -H\b;


end