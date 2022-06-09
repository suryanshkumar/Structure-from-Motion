/* *
 * Author : Suryansh Kumar, ETH Zurich 
 * 
 * */

#include <iostream>
#include <fstream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../src/basicSfM.h"

using namespace cv;
using namespace std;


int main(int argc, char *argv[]){
  
  // step 1: read the images and initialize the camera intrinsic matrix.
  Mat image_1 = imread("../../../images/0022.JPG");
  Mat image_2 = imread("../../../images/0023.JPG");
  double fx = 1698.873755; double fy = 1698.8796645; 
  double cx = 971.7497705; double cy = 647.7488275;
  
  //keep the scale factor to resize the image, if required
  //and adjust the K matrix (camera intrinsic matrix).
  int im_div_scale = 1.0;
  Mat image_ref; Mat image_nex;
  
  resize(image_1, image_ref, Size(image_1.size().width/im_div_scale, image_1.size().height/im_div_scale));
  resize(image_2, image_nex, Size(image_2.size().width/im_div_scale, image_2.size().height/im_div_scale));
  fx = fx/im_div_scale; fy = fy/im_div_scale;
  cx = cx/im_div_scale; cy = cy/im_div_scale;
  Mat K = (Mat_<double>(3, 3)<<fx, 0, cx, 0, fy, cy, 0, 0, 1);

  // execute the basic two-view structure from motion algorithm.
  basicSfM sfm;
  sfm.algorithm_sparse3d(image_ref, image_nex, K);
  
  //read the relative pose
  cout<<"Relative Rotation"<<sfm.R_rel<<endl;
  cout<<"Relative Translation"<<sfm.t_rel<<endl;
  
  //read the 3D reconstruction
  cout<<"Reconstructed 3D points"<<endl;
  vector<Point3d> reconstruction = sfm.xR;
  vector<cv::Point3d> :: iterator it = reconstruction.begin();
  while(it != reconstruction.end()){
    cout<<it->x<<" "<<it->y<<" "<<it->z<<endl;
    it++;
  }
  return 0;
}
