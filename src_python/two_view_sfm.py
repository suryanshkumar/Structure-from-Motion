#Author: Suryansh, Kumar ETH

import cv2
import numpy as np

img_w, img_h = 1944//2, 1296//2

#read and display the two images
image_1 = cv2.imread('../images/0022.JPG')
image_2 = cv2.imread('../images/0023.JPG')
ref_img = cv2.resize(image_1, (img_w, img_h))
nex_img = cv2.resize(image_2, (img_w, img_h))

cv2.imshow('reference image', ref_img)
cv2.imshow('next image', nex_img)
cv2.waitKey(0)

#get the K matrix.
fx, fy = 1698.873755, 1698.8796645
cx, cy = 971.7497705, 647.7488275
K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

# keypoint and correspondence match

# pose estimation

# 3D reconstruction

