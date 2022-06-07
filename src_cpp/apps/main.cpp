/**
 * Author : Suryansh Kumar, ETH Zurich
 * 
 * */


#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "../src/basicSfM.h"

using namespace cv;
using namespace std;


int main(int argc, char *argv[]){
	Mat image_1 = imread("../../../images/IMG_2348.JPG");
    Mat image_2 = imread("../../../images/IMG_2349.JPG");
	int im_down_scale_factor = 4;

  	basicSfM sfm;
  	sfm.algorithm_sparse3d(image_1, image_2, im_down_scale_factor);
	
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
