
/**
 * Author : Suryansh Kumar
 * Web: http://researchweb.iiit.ac.in/~suryansh
 * */

#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <iterator>
#include <string>
#include "basicSfM.h"
#include "matches.h"
#include "essential.h"
#include "triangulate.h"

using namespace std;
using namespace cv;

const double kDistanceCoef = 4.0;
const int kMaxMatchingSize = 50;


/*void basicSfM :: reconstruct_sparse3d(vector<Mat> images)
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
    //cout<<"Relative rotation"<< Est.R2c<<endl;
    //cout<<"Relative translation"<<Est.t2c<<endl;
}*/

/*void basicSfM :: algorithm_sparse3d()
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
}*/

vector<Point2f> detect_features(Mat img){
    double qualityLevel = 0.01; double minDistance = 3.0; double k = 0.04; 
    int blockSize = 3; int maxCorners = 3000; bool useHarrisDetector = true;

    vector<Point2f> img_keypoint; cv::Mat src_gray;
    cvtColor(img, src_gray, CV_BGR2GRAY);

    cv::goodFeaturesToTrack(src_gray, img_keypoint, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k );
    return img_keypoint;
}

//taken from internet
inline void match(Mat& desc1, Mat& desc2, vector<DMatch>& matches) {
    const double kDistanceCoef = 4.0;
    const int kMaxMatchingSize = 50;
    matches.clear();
    cv::BFMatcher desc_matcher(cv::NORM_L2, true);
    vector< vector<DMatch> > vmatches;
    desc_matcher.knnMatch(desc1, desc2, vmatches, 1);
    for (int i = 0; i < static_cast<int>(vmatches.size()); ++i) {
        if (!vmatches[i].size()) {
                continue;
            }
            matches.push_back(vmatches[i][0]);
        }
    
    std::sort(matches.begin(), matches.end());
    while (matches.front().distance * kDistanceCoef < matches.back().distance) {
        matches.pop_back();
    }
    while (matches.size() > kMaxMatchingSize) {
        matches.pop_back();
    }
}


void basicSfM :: reconstruct_sparse3d(vector<Mat> images)
{
    Mat ref_img = images[0]; Mat nex_img = images[1];
    Mat ref_copy; ref_img.copyTo(ref_copy); 
    Mat nex_copy; nex_img.copyTo(nex_copy);

    //detect key points and compute its descriptor
    vector<KeyPoint> ref_kpts; vector<KeyPoint> nex_kpts;
    Mat ref_desc; Mat nex_desc;
    Ptr<ORB> detector = ORB::create();

    detector->detectAndCompute(ref_img, Mat(), ref_kpts, ref_desc);
	detector->detectAndCompute(nex_img, Mat(), nex_kpts, nex_desc);

    //match the descriptor
    std::vector<DMatch> matches;
    match(ref_desc, nex_desc, matches);

    //draw the matched points
    Mat img_matches;
	drawMatches(ref_copy, ref_kpts, nex_copy, nex_kpts,
			matches, img_matches);
    
    imshow("Matches", img_matches);
	waitKey(0);
}


void basicSfM :: algorithm_sparse3d()
{
    vector<Mat> images;
    Mat image_1 = imread("../../../images/IMG_2348.JPG");
    Mat image_2 = imread("../../../images/IMG_2349.JPG");
     
    //resize the images
    Mat image_ref; Mat image_nex;
    int down_scale_factor = 12;
    resize(image_1, image_ref, Size(image_1.size().width/down_scale_factor, image_1.size().height/down_scale_factor));
    resize(image_2, image_nex, Size(image_2.size().width/down_scale_factor, image_2.size().height/down_scale_factor));

    //store the images
    images.push_back(image_ref); images.push_back(image_nex);

    reconstruct_sparse3d(images);

    //imshow("reference image", image_ref);
    //imshow("next image", image_nex);
    //waitKey(0);
}



/* Rough work
    //detect features
    vector<Point2f> ref_keypoint = detect_features(ref_img);
    vector<Point2f> nex_keypoint = detect_features(nex_img);

    //visualize the keypoints
    std::vector<Point2f> :: const_iterator it_ref = ref_keypoint.begin();
    std::vector<Point2f> :: const_iterator it_nex = nex_keypoint.begin();
    Mat ref_copy; ref_img.copyTo(ref_copy);
    Mat nex_copy; nex_img.copyTo(nex_copy);

    while(it_ref != ref_keypoint.end()) {
        circle(ref_copy, *it_ref, 1, Scalar(0, 0, 255), 2, 8, 0);
        it_ref++;
    }
    while(it_nex != nex_keypoint.end()) {
        circle(nex_copy, *it_nex, 1, Scalar(0, 255, 0), 2, 8, 0);
        it_nex++;
    }

    imshow("reference image keypoint", ref_copy);
    imshow("next image keypoint", nex_copy);
    waitKey(0);*/
