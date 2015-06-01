
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
    
  private:
  	void reconstruct_sparse3d(vector<Mat>);
  	vector<cv::Point3d> xR;
  	Mat iP1, iP2;
  	

};

#endif



