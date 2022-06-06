% Author: Suryansh Kumar, ETH

function [R_rel, t_rel, reconstructed_points] = reconstruct_3d_points(...
    x_ref, x_nex, K)

% step 1: compute fundamental matrix.
% [F, ~] = estimateFundamentalMatrix(x_ref, x_nex, Method="Norm8Point", ...
%     NumTrials=2000, DistanceThreshold=1e-4);
F = fundmatrix([x_ref'; ones(1, length(x_ref))], [x_nex'; ones(1, length(x_nex))]);

% step 2: compute essential matrix.
E = K'*F*K;

% step 3: compute possible poses for the next camera
[R1_nex, R2_nex, t1_nex, t2_nex] = compute_possible_poses(E);
R1_nex;
t1_nex;
R2_nex;
t2_nex;

% store four possible camera for the next view;
N_config = 4;            % total number of possible configuration.
pose_nex = cell(N_config, 1);
pose_nex{1} = [R1_nex, t1_nex]; % 1st possible pose
pose_nex{2} = [R1_nex, t2_nex]; % 2nd possible pose
pose_nex{3} = [R2_nex, t1_nex]; % 3rd possible pose
pose_nex{4} = [R2_nex, t2_nex]; % 4th possible pose

% step 4: triangulate image key-point correspondence using all 4 camera.
P_ref = K*eye(3, 4);     % assume the reference pose to be at origin.
scale_val = 1.0;         % homogenous approach
P_nex = cell(N_config, 1);      % total projection matrix.
X_p = cell(N_config, 1);        % reconstruction for possible cameras.
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

[consensus_val, consensus_index] = max(config_total);

% step 6: procure the correct pose for the next camera and 3D.
relative_pose =  pose_nex{consensus_index};
chiral_mat = chiral_val{consensus_index};
X_mat = X_p{consensus_index};
R_rel = relative_pose(:, 1:3);
t_rel = relative_pose(:, 4);

% step 7: homogenize to get the reconstruction in 3D.
reconstructed_points = get_filtered_3d(chiral_mat, X_mat);
end


function X = get_filtered_3d(chiral_mat, X_mat)

X = []; count = 0;
for i = 1:length(chiral_mat)
    val = chiral_mat(i); 
    if (val == 2.0)
        x = X_mat(i, 1); y = X_mat(i, 2); z = X_mat(i, 3); w = X_mat(i, 4);
        x = x/w; y = y/w; z = z/w;
        if (z > 0 && z <1000.0)
            count = count + 1;
            x_store = [x, y, z];
            X = [X; x_store];
        end
    end
end

end


% for i = 1:4
%     P_nex{i} = K*cam_nex{i};
% end
% P_nex{1} = K*cam1_nex; P_nex{2} = K*cam2_nex; 
% P_nex{3} = K*cam3_nex; P_nex{4} = K*cam4_nex; 

% P1_nex = K*[R1_nex, t1_nex]; % 1st possible camera
% P2_nex = K*[R1_nex, t2_nex]; % 2nd possible camera
% P3_nex = K*[R2_nex, t1_nex]; % 3rd possible camera
% P4_nex = K*[R2_nex, t2_nex]; % 4th possible camera

% X1_p = triangulate_points(x_ref, x_nex, scale, P_ref, P1_nex);
% X2_p = triangulate_points(x_ref, x_nex, scale, P_ref, P2_nex);
% X3_p = triangulate_points(x_ref, x_nex, scale, P_ref, P3_nex);
% X4_p = triangulate_points(x_ref, x_nex, scale, P_ref, P4_nex);

% chiral1_val = compute_chiral_estimate(P_ref, P1_nex, X1_p);
% chiral2_val = compute_chiral_estimate(P_ref, P2_nex, X2_p);
% chiral3_val = compute_chiral_estimate(P_ref, P3_nex, X3_p);
% chiral4_val = compute_chiral_estimate(P_ref, P4_nex, X4_p);

% config1_total = sum(chiral1_val);
% config2_total = sum(chiral2_val);
% config3_total = sum(chiral3_val);
% config4_total = sum(chiral4_val);
% 
% [consensus_val, consensus_index] = max([config1_total, ...
%     config2_total, config3_total, config4_total]);
