/* Author : Suryansh Kumar, ETH Zurich */

#include <iostream>
#include <iterator>
#include <string>
#include <fstream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core.hpp"
#include "basicSfM.h"
#include "essential.h"
#include "triangulate.h"

using namespace std;
using namespace cv;


void basicSfM :: reconstruct_sparse3d(){
  essential Est;

  //Step 1: Compute Essential Matrix.
  K_mat.copyTo(Est.K);
  Est.computeEssentialMat(ref_keypoints, nex_keypoints);

  //Step 2. Decomposition of E into four possible camera poses;
  Est.computePose();

  //Step 3. Reconstruct for possible Rt matices using Triangulation;
  triangulate tr; float scale = 1.0;
  Mat Xn1 = tr.triangulate_points(ref_keypoints, nex_keypoints, scale, Est.P0, Est.P1);
  Mat Xn2 = tr.triangulate_points(ref_keypoints, nex_keypoints, scale, Est.P0, Est.P2); 
  Mat Xn3 = tr.triangulate_points(ref_keypoints, nex_keypoints, scale, Est.P0, Est.P3);
  Mat Xn4 = tr.triangulate_points(ref_keypoints, nex_keypoints, scale, Est.P0, Est.P4);

  //Step 4. Check the chirality to validate the reconstruction;
  Est.check_chirality(Xn1, Xn2, Xn3, Xn4);

  //store the relative pose, projection matrix and reconstruction.
  (Est.R2c).copyTo(R_rel); (Est.t2c).copyTo(t_rel); (Est.P2c).copyTo(P_rel);
  xR = Est.xReconstructed;

}

vector< DMatch > basicSfM :: match_keypoint_descriptors(Mat descriptors_ref, 
                                                        Mat descriptors_nex){
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match(descriptors_ref, descriptors_nex, matches);
  
  double max_dist = 0; double min_dist = 100;
  for(int i = 0; i < descriptors_ref.rows; i++){
    double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  
  //printf("-- Max dist : %f \n", max_dist);
  //printf("-- Min dist : %f \n", min_dist);
  
  std::vector< DMatch > filtered_matches;
  for(int i = 0; i < descriptors_ref.rows; i++){
    if(matches[i].distance <= max(2*min_dist, 0.10)){
      filtered_matches.push_back(matches[i]);
      }
  }
  return filtered_matches;
}


void basicSfM :: estimate_keypoint_correspondences(vector<Mat> images){
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

vector<Point2f> basicSfM::read_keypoints_from_txt_file(string filename){
  vector<Point2f> keypoints; vector<vector<float>> data;
  ifstream infile(filename);
  string line; string str;
  
  //Read the file
  while (getline(infile, line)){
    istringstream ss(line);
    vector<float> record;
    while (getline(ss, str, ',')){
      record.push_back(std::stod(str));
      }
    data.push_back(record);
    }
    
    for (size_t i = 0; i < data.size(); i++){
      vector<float> record;
      record = data[i];
      keypoints.push_back(Point2f(record[0], record[1]));
    }
  return keypoints;
}


Mat basicSfM :: drawCorrespondances(){
  Mat src_copy;
  image_ref.copyTo(src_copy);
  
  std::vector<Point2f> :: const_iterator itc = ref_keypoints.begin();
  std::vector<Point2f> :: const_iterator itf = nex_keypoints.begin();
  int count = 0;
  
  while(itc!=ref_keypoints.end()){
    circle(src_copy, *itc, 1, Scalar(0, 0, 255), 2, 8, 0);
    circle(src_copy, *itf, 1, Scalar(0, 255, 0), 2, 8, 0);
    cv::line(src_copy, *itc, *itf, Scalar(0, 255, 255) );
    itc++; itf++; count++;
  }
  return src_copy;
}

void basicSfM :: algorithm_sparse3d(Mat image_1, Mat image_2, Mat K){
  //save and display the images
  image_1.copyTo(image_ref); image_2.copyTo(image_nex);
  
  vector<Mat> images;
  images.push_back(image_ref); images.push_back(image_nex);
  imshow("reference image", image_ref);
  imshow("next image", image_nex);
  waitKey(0);
  
  //estimate the keypoint and match its correspondences in the next image
  
  //uncomment the next line to automate the keypoiny detection and matching
  /*estimate_keypoint_correspondences(images);*/
  
  //use the saved files to read the SIFT keypoint correspondences.
  //opencv SIFT features requires non-free modules to be installed.
  //Therefore, I saved the SIFT features from MATLAB 2022 for this example code.
  ref_keypoints = read_keypoints_from_txt_file("../../../src_cpp/src/SIFT_KeyPoints/keypoint_ref.txt");
  nex_keypoints = read_keypoints_from_txt_file("../../../src_cpp/src/SIFT_KeyPoints/keypoint_nex.txt");
  img_matches = drawCorrespondances();
  
  //display the image keypoint correspondences
  imshow("Matches", img_matches);
  waitKey(0);
  
  //reconstruct the matched point correspondences.
  K.copyTo(K_mat);
  reconstruct_sparse3d();
}
