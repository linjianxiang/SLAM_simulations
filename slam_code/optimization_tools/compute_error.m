% Computes the total error of the graph
function Fx = compute_error(x1,x2,edge)

Fx = 0;

Z = v2t(edge);
e12 = t2v(invt(Z)*(invt(x1)*x2));
Fx = Fx + e12'*e12;


end
