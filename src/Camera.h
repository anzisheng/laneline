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
	cv::Point2f Wcs2Ics(cv::Point3f point3d, double disparity);
	void setRT2NextFrame(std::string matFile);
private:
	arma::mat intrinsic;
	arma::mat extrinsic;
	arma::mat lw;
	double zc;
	double baseline;
	arma::mat frameRT; //从当前帧到下一帧的RT。
};

