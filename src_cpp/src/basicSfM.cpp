
/**
 * Author : Suryansh Kumar
 * Web: http://researchweb.iiit.ac.in/~suryansh
 * */

#include <iostream>
#include <iterator>
#include <string>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core.hpp"
#include "basicSfM.h"
#include "matches.h"
#include "essential.h"
#include "triangulate.h"

using namespace std;
using namespace cv;


void basicSfM :: reconstruct_sparse3d(){

    essential Est;
    //Est.getIntrinsic(im_scale_factor);
    K_mat.copyTo(Est.K);
    Est.computeEssentialMat(ref_keypoints, nex_keypoints);

    //cout<<"Step1. Decomposition of E into four possible camera poses"<<endl;
  
    Est.computePose();
  
    //cout<<"Step2. Reconstruct for possible Rt matices using Triangulation" <<endl;
    triangulate tr; float scale = 1.0;
  
    Mat Xn1 = tr.triangulate_points(ref_keypoints, nex_keypoints, scale, Est.P0, Est.P1);
    Mat Xn2 = tr.triangulate_points(ref_keypoints, nex_keypoints, scale, Est.P0, Est.P2);
    Mat Xn3 = tr.triangulate_points(ref_keypoints, nex_keypoints, scale, Est.P0, Est.P3);
    Mat Xn4 = tr.triangulate_points(ref_keypoints, nex_keypoints, scale, Est.P0, Est.P4);
  
    //cout<<"Step3. Check the chirality to validate the reconstruction"<<endl;
    Est.check_chirality(Xn1, Xn2, Xn3, Xn4);

    //store the relative pose, projection matrix and reconstruction.
    (Est.R2c).copyTo(R_rel); (Est.t2c).copyTo(t_rel); (Est.P2c).copyTo(P_rel);
    xR = Est.xReconstructed;
}

vector< DMatch > basicSfM :: match_keypoint_descriptors(Mat descriptors_ref, Mat descriptors_nex){

    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match(descriptors_ref, descriptors_nex, matches);

    double max_dist = 0; double min_dist = 100;
    
    for( int i = 0; i < descriptors_ref.rows; i++ ){ 
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    std::vector< DMatch > filtered_matches;
    for(int i = 0; i < descriptors_ref.rows; i++ ){ 
        if( matches[i].distance <= max(2*min_dist, 0.10) ){
            filtered_matches.push_back(matches[i]);
        }
    }

    return filtered_matches;
}


void basicSfM :: estimate_keypoint_correspondences(vector<Mat> images)
{
    Mat ref_img = images[0]; Mat nex_img = images[1];
    Mat ref_copy; ref_img.copyTo(ref_copy); 
    Mat nex_copy; nex_img.copyTo(nex_copy);

    //detect key points and compute its descriptor
    vector<KeyPoint> ref_kpts; vector<KeyPoint> nex_kpts;
    Mat ref_desc; Mat nex_desc;
    Ptr<KAZE> detector = KAZE::create();

    detector->detectAndCompute(ref_img, Mat(), ref_kpts, ref_desc);
	detector->detectAndCompute(nex_img, Mat(), nex_kpts, nex_desc);

    matches = match_keypoint_descriptors(ref_desc, nex_desc);

	drawMatches(ref_copy, ref_kpts, nex_copy, nex_kpts, 
        matches, img_matches, Scalar::all(-1), Scalar::all(-1),
        vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    
    for (int i = 0; i<(int)matches.size(); i++){
        ref_keypoints.push_back(ref_kpts[matches[i].queryIdx].pt);
        nex_keypoints.push_back(nex_kpts[matches[i].trainIdx].pt);
    }
}


void basicSfM :: algorithm_sparse3d(Mat image_ref, Mat image_nex, Mat K)
{
    //display the images
    imshow("reference image", image_ref);
    imshow("next image", image_nex);
    waitKey(0);

    //store the images
    vector<Mat> images;
    images.push_back(image_ref); images.push_back(image_nex);

    //estimate the keypoint and match its correspondences in the next image
    estimate_keypoint_correspondences(images);

    //display the image keypoint correspondences
    imshow("Matches", img_matches);
    waitKey(0);

    //reconstruct the matched point correspondences.
    K.copyTo(K_mat);
    reconstruct_sparse3d();
}
