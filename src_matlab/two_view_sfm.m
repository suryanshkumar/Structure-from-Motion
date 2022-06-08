% Author: Suryansh Kumar, ETH Zurich

% basic steps for two-view structure from motion
addpath(genpath("./mvg_modules"));

% % step 1: read the two images and initialize camera intrinsic parameters.
I_ref = imread("../images/0022.JPG");
I_nex = imread("../images/0023.JPG");
fx = 1698.873755; fy = 1698.8796645;
cx = 971.7497705; cy = 647.7488275;

% Note: MATLAB inbuild feature descriptor match may fail at very high image 
% resolution. Thus, we may have to resized it and appropritely change
% the K matrix values. For example: on IMG_2348.JPG IMG_2349.JPG, 
% IMG_2356.JPG, IMG_2357.JPG choose im_div_scale = 4.0;
[m, n, ~] = size(I_ref);
im_div_scale = 1.0;
I_ref = imresize(I_ref, [m/im_div_scale, n/im_div_scale]);
I_nex = imresize(I_nex, [m/im_div_scale, n/im_div_scale]);
fx = fx/im_div_scale; fy = fy/im_div_scale;
cx = cx/im_div_scale; cy = cy/im_div_scale;
K_mat = [fx, 0, cx; 0, fy, cy; 0, 0, 1];

% % step 2: compute the key point in the two images.
corner_ref = detectSIFTFeatures(rgb2gray(I_ref));
corner_nex = detectSIFTFeatures(rgb2gray(I_nex));
[feature_ref, valid_ref] = extractFeatures(rgb2gray(I_ref), corner_ref);
[feature_nex, valid_nex] = extractFeatures(rgb2gray(I_nex), corner_nex);

figure;
subplot(1, 2, 1); imshow(I_ref); title('First image keypoints'); hold on; 
plot(valid_ref.Location(:, 1), valid_ref.Location(:, 2), 'r.');
subplot(1, 2, 2), imshow(I_nex); title('Second image keypoints'); hold on; 
plot(valid_nex.Location(:, 1), valid_nex.Location(:, 2), 'g+');

% % step 3: match the key points.
indexPairs = matchFeatures(feature_ref, feature_nex);
matchedpoint_ref = valid_ref(indexPairs(:, 1), :);
matchedpoint_nex = valid_nex(indexPairs(:, 2), :);

figure; hold on ; title('Keypoint correspondence');
showMatchedFeatures(I_ref, I_nex, matchedpoint_ref, matchedpoint_nex, ...
    'montage');


% % step 4: recover relative pose and 3D.
 [R, t, reconstructed_3d_points] = reconstruct_3d_points(...
     matchedpoint_ref.Location, matchedpoint_nex.Location, K_mat);

% % step 5: plot the reconstruction.
% MATLAB 3D visualizer is a big-time disappointment, so better store the 
% reconstructed_3d_points and visualize the points in a 
% good 3D visualizer. Recommended software package: pangolin, open3d.

% figure; hold on;
% plot3(reconstructed_3d_points(:, 1), reconstructed_3d_points(:, 2), ...
%    reconstructed_3d_points(:, 3), 'k.');





