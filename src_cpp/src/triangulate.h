/* Author : Suryansh Kumar*/

#ifndef TRIANGULATE_H
#define TRIANGULATE_H

#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class triangulate{
  public:
    Mat triangulate_points(vector<Point2f>, vector<Point2f>, float, Mat, Mat);
  };

#endif
