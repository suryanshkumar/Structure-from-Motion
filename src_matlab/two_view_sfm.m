%Author: Suryansh Kumar, ETH

% basic steps for two-view structure from motion
addpath(genpath("./mvg_modules"));

% % step 1: read the two image and K matrix.
I_ref = imread("images/IMG_2348.JPG");
I_nex = imread("images/IMG_2349.JPG");

% MATLAB inbuild feature descriptor match may fail at original image 
% resolution. Thus, we resized it and appropritely changed the K matrix.
[m, n, ~] = size(I_ref);
im_scale = 4.0;

I_ref = imresize(I_ref, [m/im_scale, n/im_scale]);
I_nex = imresize(I_nex, [m/im_scale, n/im_scale]);
fx = 3838.27/im_scale; fy = 3837.22/im_scale; 
cx = 2808.00/im_scale; cy = 1872.00/im_scale;
K_mat = [fx, 0, cx; 0, fy, cy; 0, 0, 1];

% % step 2: compute the key point in the two images.
corner_ref = detectORBFeatures(rgb2gray(I_ref));
corner_nex = detectORBFeatures(rgb2gray(I_nex));
[feature_ref, valid_ref] = extractFeatures(rgb2gray(I_ref), corner_ref);
[feature_nex, valid_nex] = extractFeatures(rgb2gray(I_nex), corner_nex);

figure,
subplot(1, 2, 1); imshow(I_ref); hold on; 
plot(valid_ref.Location(:, 1), valid_ref.Location(:, 2), 'r.');
subplot(1, 2, 2), imshow(I_nex); hold on; 
plot(valid_nex.Location(:, 1), valid_nex.Location(:, 2), 'g+');

% step 3: match the key points.
indexPairs = matchFeatures(feature_ref, feature_nex);
matchedpoint_ref = valid_ref(indexPairs(:, 1), :);
matchedpoint_nex = valid_nex(indexPairs(:, 2), :);

figure, 
showMatchedFeatures(I_ref, I_nex, matchedpoint_ref, matchedpoint_nex, ...
    'montage');

% % step 4: recover relative pose and 3D.
 [R, t, reconstructed_3d_points] = reconstruct_3d_points(...
     matchedpoint_ref.Location, matchedpoint_nex.Location, K_mat);


% % step 5: plot the reconstruction.
% MATLAB 3D plotting sucks big-time, so better store the 
% reconstructed_3d_points and visualize the points in a 
% good 3D visualizer. Recommended software: pangolin.
%figure, hold on;
%plot3(reconstructed_3d_points(:, 1), reconstructed_3d_points(:, 2), reconstructed_3d_points(:, 3), 'k.');





