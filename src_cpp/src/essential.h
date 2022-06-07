/**
 * Author : Suryansh Kumar
 * Web: http://researchweb.iiit.ac.in/~suryansh
 * */


#ifndef ESSENTIAL_H
#define ESSENTIAL_H
#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class essential
{
   public:
   void getIntrinsic(int); 
   void computeEssentialMat(vector<Point2f>, vector<Point2f>);
   void computePose();
   void check_chirality(Mat, Mat, Mat, Mat);
   Mat P0, P1, P2, P3, P4;
   Mat P2c, R2c, t2c;
   vector<Point3d> xReconstructed;

   protected:
      Mat K, F, E, R1, R2, t1, t2; 
  
   private:
   	  void get_valid_3d(Mat, Mat, Mat, Mat, Mat);
   	  Mat sign(Mat );
};


#endif
