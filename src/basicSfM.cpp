
/**
 * Author : Suryansh Kumar
 * Web: http://researchweb.iiit.ac.in/~suryansh
 * */

#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <string>
#include "basicSfM.h"
#include "matches.h"
#include "essential.h"
#include "triangulate.h"

using namespace std;
using namespace cv;


void basicSfM :: reconstruct_sparse3d(vector<Mat> images)
{

    //cout<<"Step1. Compute correspondance match between images "<<endl;
    matches Mt;
    
    Mat image1 = images[0];
    Mat image2 = images[1];
  
    Mt.computeCorrespondances(image1, image2, false);  
    vector<Point2f> iF1 = Mt.i1_features;

    for(int i = 2; i<images.size(); i++)
    {
        images[i].copyTo(image2);
        Mt.i1_features = Mt.i2_features;
        Mt.computeCorrespondances(image1, image2, true);
    }

    vector<Point2f> iF2 = Mt.i2_features;
    Mat tracks = Mt.drawCorrespondances();
    
    imshow("tracks", tracks);
    waitKey(0);
  
    //cout<<"Step2. Compute  Essential Matrix "<<endl;
  
    essential Est;
    Est.getIntrinsic();
    Est.computeEssentialMat(iF1, iF2);
  
  
    //cout<<"Step3. Decomposition of E into four possible camera poses"<<endl;
  
    Est.computePose();
  
  
  
    //cout<<"Step4. Reconstruct for possible Rt matices using Triangulation" <<endl;
    triangulate tr; float scale = 1.0;
  
    Mat Xn1 = tr.triangulate_points(iF1, iF2, scale, Est.P0, Est.P1);
    Mat Xn2 = tr.triangulate_points(iF1, iF2, scale, Est.P0, Est.P2);
    Mat Xn3 = tr.triangulate_points(iF1, iF2, scale, Est.P0, Est.P3);
    Mat Xn4 = tr.triangulate_points(iF1, iF2, scale, Est.P0, Est.P4);
  
    //cout<<"Step5. Check the chirality to validate the reconstruction"<<endl;
    Est.check_chirality(Xn1, Xn2, Xn3, Xn4);
    vector<cv::Point3d> xR = Est.xReconstructed;
    vector<cv::Point3d> :: iterator it = xR.begin();
  
    while(it != xR.end())
    {
        cout<<it->x<<" "<<it->y<<" "<<it->z<<endl;
        it++;
    }
    (Est.P2c).copyTo(iP1);
   
}

void basicSfM :: algorithm_sparse3d()
{
    vector<Mat> images;
    for(int i = 15; i<=19; i++)
    {
      stringstream ss;
      ss<<i;
      string iPath = "../../images/" + ss.str() + ".pgm";
      Mat image = imread(iPath, 1);
      images.push_back(image);
    }
    
    reconstruct_sparse3d(images);
}
