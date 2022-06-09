/* Author: Suryansh Kumar */

#ifndef BASICSFM_H
#define BASICSFM_H

#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;


class basicSfM{
  public:
   Mat image_ref; Mat image_nex;
   Mat R_rel, t_rel, P_rel;
   vector<cv::Point3d> xR;
   
   void algorithm_sparse3d(Mat, Mat, Mat);

  private:
    vector<Point2f> ref_keypoints;
    vector<Point2f> nex_keypoints;
    vector< DMatch > matches;
    Mat K_mat;
    Mat img_matches;
    int im_scale_factor;
    
    void reconstruct_sparse3d();
    void estimate_keypoint_correspondences(vector<Mat>);
    vector<DMatch> match_keypoint_descriptors(Mat, Mat);
    vector<Point2f> read_keypoints_from_txt_file(string filename);
    Mat drawCorrespondances();
};

#endif
