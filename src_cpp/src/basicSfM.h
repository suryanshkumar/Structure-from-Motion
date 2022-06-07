
/**
 * Author : Suryansh Kumar
 * Web: http://researchweb.iiit.ac.in/~suryansh
 * */

#ifndef BASICSFM_H
#define BASICSFM_H

#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;


class basicSfM
{
  public: 
  	void algorithm_sparse3d();
	void estimate_keypoint_correspondences(vector<Mat> images);
    
  private:
  	void reconstruct_sparse3d();
  	vector<cv::Point3d> xR;
	vector<Point2f> ref_keypoints;
    vector<Point2f> nex_keypoints;
  	Mat iP1, iP2;
	int down_scale_factor;
};

#endif



