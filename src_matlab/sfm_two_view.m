%Author: Suryansh Kumar, ETH

% basic steps for two-view structure from motion

% step 1: read the two image and K matrix.
I_ref = imread("../images/15.pgm");
I_nex = imread("../images/16.pgm");
K_mat = load("K_matrix.txt");

% step 2: compute the key point in the two images.
corner_ref = detectSIFTFeatures(I_ref);
corner_nex = detectSIFTFeatures(I_nex);
[feature_ref, valid_ref] = extractFeatures(I_ref, corner_ref);
[feature_nex, valid_nex] = extractFeatures(I_nex, corner_nex);

figure,
subplot(1, 2, 1); imshow(I_ref); hold on; 
plot(valid_ref.Location(:, 1), valid_ref.Location(:, 2), 'ro');
subplot(1, 2, 2), imshow(I_nex); hold on; 
plot(valid_nex.Location(:, 1), valid_nex.Location(:, 2), 'g+');

% step 3: match the key points.
indexPairs = matchFeatures(feature_ref, feature_nex);
matchedpoint_ref = valid_ref(indexPairs(:, 1), :);
matchedpoint_nex = valid_nex(indexPairs(:, 2), :);

figure, 
showMatchedFeatures(I_ref, I_nex, matchedpoint_ref, matchedpoint_nex);

% step 4: recover relative pose and 3D.
[R, t, reconstructed_3d_points] = reconstruct_3d_points(...
    matchedpoint_ref.Location, matchedpoint_nex.Location, K_mat);



