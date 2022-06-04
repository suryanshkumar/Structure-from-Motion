%Author: Suryansh Kumar, ETH

function reconstructed_points = reconstruct_3d_points(x_ref, x_nex, K)

reconstructed_points = [];

%% step 1: compute fundamental matrix.
[F, ~] = estimateFundamentalMatrix(x_ref, x_nex, Method="Norm8Point", ...
    NumTrials=2000,DistanceThreshold=1e-4);


%% step 2: compute essential matrix.
E = K'*F*K;


%% step 3: compute possible poses for the next camera
[R1_nex, R2_nex, t1_nex, t2_nex] = compute_possible_poses(E, K);

% four possible camera;
P_nex1 = K*[R1_nex, t1_nex]; P_nex2 = K*[R1_nex, t2_nex];
P_nex3 = K*[R2_nex, t1_nex]; P_nex3 = K*[R2_nex, t2_nex];

% assume the reference pose to at origin.
P_ref = K*eye(3, 4);
 
%% step 4: perform the traingulation for all the four camera.

end