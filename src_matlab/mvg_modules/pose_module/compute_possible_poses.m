% Author: Suryansh Kumar, ETH
% Refer Hartley and Zisserman Multiple View Geometry Book
% page 258-259, Result 9.18-9.19.

function [R1, R2, t1, t2] = compute_possible_poses(E)

W = [0, -1, 0; 1, 0, 0; 0, 0, 1];
[U, ~, V] = svd(E);

% the two possible rotation
R1 = U*W*V';
R2 = U*W'*V';

% the two possible translation
t1 = U(:, 3);
t2 = -U(:, 3);

% check the determinant for a valid rotation and correct for reflection.
if (det(R1) < 0)
    R1 = -R1;
end

if (det(R2) < 0)
    R2 = -R2;
end

end