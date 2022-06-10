/*Author : Suryansh Kumar*/

#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "triangulate.h"

using namespace std;
using namespace cv;

Mat triangulate::triangulate_points(vector<Point2f> f1, vector<Point2f> f2, 
                                    float scale, Mat P2, Mat P1){
  Mat J(4, 4, CV_32F, Scalar(0));
  Mat X(f1.size(), 4, CV_32F, Scalar(0));

  for (int i = 0; i < f1.size(); i++){
    float x1 = f1[i].x / scale; float y1 = f1[i].y / scale;
    float x2 = f2[i].x / scale; float y2 = f2[i].y / scale;

    Mat t1 = (P1.row(2)).mul(x1) - P1.row(0); t1.copyTo(J.row(0));
    Mat t2 = (P1.row(2)).mul(y1) - P1.row(1); t2.copyTo(J.row(1));
    Mat t3 = (P2.row(2)).mul(x2) - P2.row(0); t3.copyTo(J.row(2));
    Mat t4 = (P2.row(2)).mul(y2) - P2.row(1); t4.copyTo(J.row(3));

    //Finding the solution with minimum error
    SVD svd(J);
    Mat V = svd.vt.t();
    Mat t = V.col(3);
    t = t.t(); t.copyTo(X.row(i));
  }
  return X;
}
