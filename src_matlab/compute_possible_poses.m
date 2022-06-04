% Author: Suryansh Kumar, ETH

function [R1, R2, t1, t2] = compute_possible_poses(E, K)

W = [0, -1, 0; 1, 0, 0; 0, 0, 1];
Z = [0, 1, 0; -1, 0, 0; 0, 0, 0];

[U, S, V] = svd(E);

% the two possible rotation
R1 = U*W*V';
R2 = U*W'*V';

% the two possible translation
t1 = U(:, 2);
t2 = -U(:, 2);

% check the determinant for a valid rotation and correct for reflection.
if (det(R1) < 0)
    R1 = -R1;
end

if (det(R2) < 0)
    R2 = -R2;
end

end