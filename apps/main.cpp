/**
 * Author : Suryansh Kumar
 * Web: http://researchweb.iiit.ac.in/~suryansh
 * 
 * */


#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "../src/basicSfM.h"

using namespace cv;
using namespace std;


int main(int argc, char *argv[])
{

  	basicSfM sfm;
  	
  	sfm.algorithm_sparse3d();
  	
  	return 0;
}
