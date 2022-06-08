% Author: Suryansh Kumar, ETH Zurich

function [R_rel, t_rel, reconstructed_points] = reconstruct_3d_points(...
    x_ref, x_nex, K)

% step 1: compute fundamental matrix 
% Used Peter Kovesi script for estimation fundamental matrix.
F = fundmatrix([x_ref'; ones(1, length(x_ref))], ...
    [x_nex'; ones(1, length(x_nex))]);

% step 2: compute essential matrix.
E = K'*F*K;

% step 3: compute possible poses for the next camera
% Refer Hartley and Zisserman Multiple View Geometry Book
% page 258-259, Result 9.18-9.19.

[R1_nex, R2_nex, t1_nex, t2_nex] = compute_possible_poses(E);

% store four possible camera for the next view;
N_config = 4;            % total number of possible configuration.
pose_nex = cell(N_config, 1);
pose_nex{1} = [R1_nex, t1_nex]; % 1st possible pose
pose_nex{2} = [R1_nex, t2_nex]; % 2nd possible pose
pose_nex{3} = [R2_nex, t1_nex]; % 3rd possible pose
pose_nex{4} = [R2_nex, t2_nex]; % 4th possible pose

% step 4: triangulate image key-point correspondence using all 4 camera.
P_ref = K*eye(3, 4);     % assuming the reference pose to be at origin.
scale_val = 1.0;         % homogenous approach

P_nex = cell(N_config, 1);      % total number of projection matrix.
X_p = cell(N_config, 1);        % 3D reconstruction using possible cameras.
chiral_val = cell(N_config, 1); % chiral value corresponding to cameras.

for i = 1:N_config
    %get the projection matrix corresponding to next camera.
    P_nex{i} = K*pose_nex{i};
    
    %triangulate points.
    X_p{i} = triangulate_points(x_ref, x_nex, scale_val, P_ref, P_nex{i});
    
    %get chirality values using the unnormalized reconstructed points.
    chiral_val{i} = compute_chiral_estimate(P_ref, P_nex{i}, X_p{i});
end

% step 5: check which possible camera has the maximum chiral values.
config_total = zeros(N_config, 1);
for  i = 1:N_config
    config_total(i, 1) = sum(chiral_val{i});
end

% max of chiral value assures useful pose configuration for reconstruction
[~, consensus_index] = max(config_total);

% step 6: procure the correct pose for the next camera and its 3D.
relative_pose =  pose_nex{consensus_index};
chiral_mat = chiral_val{consensus_index};
X_mat = X_p{consensus_index};
R_rel = relative_pose(:, 1:3);
t_rel = relative_pose(:, 4);

% step 7: homogenize to get the 3D reconstruction and filter numerically
% unstable reconstruction via simple threshold.
reconstructed_points = filter_and_recover(chiral_mat, X_mat);
end


% get the x/w, y/w, z/w from x, y, z, w.
function X = filter_and_recover(chiral_mat, X_mat)
X = []; count = 0;
FILTER_THR = 20.0; % threshold z value for having good reconstruction.

for i = 1:length(chiral_mat)
    val = chiral_mat(i); 
    if (val == 2.0)
        x = X_mat(i, 1); y = X_mat(i, 2); z = X_mat(i, 3); w = X_mat(i, 4);
        x = x/w; y = y/w; z = z/w;
        if (z > 0 && z <FILTER_THR)
            count = count + 1;
            x_store = [x, y, z];
            X = [X; x_store];
        end
    end
end

end