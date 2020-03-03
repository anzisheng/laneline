#pragma once
#include <armadillo>
//#include <opencv>
#include <opencv2/opencv.hpp>
//https://blog.csdn.net/zhou4411781/article/details/103876478
using namespace arma;
using namespace cv;
class Camera
{
public:
	Camera();
	cv::Point3f Ics2Wcs(cv::Point2f point2d, double disparity);
private:
	arma::mat intrinsic;
	arma::mat extrinsic;
	arma::mat lw;
	double zc;
	double baseline;

};

