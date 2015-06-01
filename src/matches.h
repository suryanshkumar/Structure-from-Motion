/**
 * Author : Suryansh Kumar
 * Web: http://researchweb.iiit.ac.in/~suryansh
 * 
 * */




#ifndef MATCHES_H
#define MATCHES_H
#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class matches
{
  public: 
  
    void computeCorrespondances(Mat, Mat, bool);
    Mat  drawCorrespondances();
    Mat  i1, i2;
    Mat denseflow;
    vector<Point2f> i1_features;
    vector<Point2f> i2_features;
    vector<Mat> i1_colors;
    
  private:
  	void detectfeature();
  	void matchFeatures();
  	void filterFeatures();
    double qualityLevel; double minDistance;
    int blockSize; bool useHarrisDetector;
    double k;
    int maxCorners;
};

#endif
