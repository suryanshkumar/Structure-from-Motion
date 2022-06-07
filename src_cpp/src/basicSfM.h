
/**
 * Author : Suryansh Kumar
 * Web: http://researchweb.iiit.ac.in/~suryansh
 * */

#ifndef BASICSFM_H
#define BASICSFM_H

#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;


class basicSfM
{
  public:
   Mat image_ref; Mat image_nex;
   Mat R_rel, t_rel, P_rel;
   vector<cv::Point3d> xR;

   void algorithm_sparse3d(Mat, Mat, int);

  private:
   vector<Point2f> ref_keypoints;
   vector<Point2f> nex_keypoints;
   vector< DMatch > matches;
   int im_scale_factor;
   
   void reconstruct_sparse3d();
   void estimate_keypoint_correspondences(vector<Mat>);
   vector<DMatch> match_keypoint_descriptors(Mat, Mat);
};

#endif



