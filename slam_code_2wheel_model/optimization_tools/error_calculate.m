function c = error_calculate(constraints, odom)
c = 0;
for i = 1:(size(constraints.pose_pose_edges,1))
    index = constraints.pose_pose_edges(i,:);
    r1 =index(1);
    r2 =index(2);
    x1 = v2t(odom(:,r1));
    x2 = v2t(odom(:,r2));
    c = c + compute_error(x1,x2,constraints.z(i,:));
end


end