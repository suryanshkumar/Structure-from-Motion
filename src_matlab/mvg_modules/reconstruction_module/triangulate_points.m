% Author: Suryansh Kumar, ETH

% Input: 
% x_ref: reference image 2d key-points.
% x_nex: next image corresponding points.
% scale: for homogeneous it is set to 1.
% P_ref: reference image projection matrix.
% P_nex: next image projection matrix.

function X = triangulate_points(x_ref, x_nex, scale, P_ref, P_nex)

[number_of_points, ~] = size(x_ref);

X = zeros(4, number_of_points);

for i = 1:number_of_points
    ui_ref = x_ref(i, 1)/scale; vi_ref = x_ref(i, 2)/scale;
    ui_nex = x_nex(i, 1)/scale; vi_nex = x_nex(i, 2)/scale;

    A = zeros(4, 4);
    A(1, :) = P_ref(3, :)*ui_ref - P_ref(1, :);
    A(2, :) = P_ref(3, :)*vi_ref - P_ref(2, :);
    A(3, :) = P_nex(3, :)*ui_nex - P_nex(1, :);
    A(4, :) = P_nex(3, :)*vi_nex - P_nex(2, :);
    [~, ~, V] = svd(A);
    X(:, i) = V(:, 4);
end

X = X'; % making it into the form (number of point, 4)

end