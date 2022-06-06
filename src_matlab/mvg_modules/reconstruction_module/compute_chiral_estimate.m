% Author: Suryansh Kumar, ETH


function chiral_val = compute_chiral_estimate(P1, P2, X)

X = X'; % for matrix multiplication
p1x = P1*X; %projection to first camera.
p2x = P2*X; %projection to second camera.

chiral_val = (sign(p1x(3,:) ) .* sign (X(4,:))) +  ...
    (sign(p2x(3,:) ) .* sign (X(4,:)));

end

