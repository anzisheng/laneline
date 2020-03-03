#include "Camera.h"

Camera::Camera()
{
	intrinsic = mat(3, 4, fill::zeros);
	//data: [1.7152766061605544e+03, 0., 9.6131084882704909e+02, 0.,
	//1.7152766061605544e+03, 5.8104840506995652e+02, 0., 0., 1.]
	intrinsic(0, 0) = 1.7152766061605544e+03;//
	intrinsic(0, 2) = 9.6131084882704909e+02;//
	intrinsic(1, 1) = 1.7152766061605544e+03;//
	intrinsic(1, 2) = 5.8104840506995652e+02;//
	intrinsic(2, 2) = 1;
	intrinsic.print("intrinsic");

	//arma::mat K_inv = inv(K);
	extrinsic = mat(4, 4, fill::zeros);
	/*[9.9999918749723515e-01, 1.2112250813246447e-03,
		-3.9741498709294705e-04, -1.2137815176612859e-03,
		9.9997815927536660e-01, -6.4967458528765289e-03,
		3.8953728573763664e-04, 6.4972229492187361e-03,
		9.9997881695296476e-01]*/
	extrinsic(0, 0) = 9.9999918749723515e-01;
	extrinsic(0, 1) = 1.2112250813246447e-03;
	extrinsic(0, 2) = -3.9741498709294705e-04;

	extrinsic(1, 0) = -1.2137815176612859e-03;
	extrinsic(1, 1) = 9.9997815927536660e-01;
	extrinsic(1, 2) = -6.4967458528765289e-03;

	extrinsic(2, 0) = 3.8953728573763664e-04;
	extrinsic(2, 1) = 6.4972229492187361e-03;
	extrinsic(2, 2) = 9.9997881695296476e-01;
	//-2.9481284557896043e+02, -1.0228736281267095e+00,	-4.5717888581795005e+00
	extrinsic(3, 0) = -2.9481284557896043e+02;
	extrinsic(3, 1) = -1.0228736281267095e+00;
	extrinsic(3, 2) = -4.5717888581795005e+00;
	extrinsic(3, 3) = 1;
	extrinsic.print("extrinsic");
	
	lw = mat(3, 4, fill::zeros);
	lw = intrinsic * extrinsic;//affmul(intrinsic, extrinsic);//
	lw.print("lw");

	baseline = sqrt((9.6131084882704909e+02 - 9.6272421647124725e+02)*(9.6131084882704909e+02 - 9.6272421647124725e+02)
	+(5.8104840506995652e+02- 5.9145511057942895e+02)*(5.8104840506995652e+02 - 5.9145511057942895e+02));
	//zc = extrinsic(0, 0)*baseline / disparity;
}

cv::Point3f Camera::Ics2Wcs(cv::Point2f point2d, double disparity)
{
	double zc = extrinsic(0, 0)*baseline / disparity;
	cv::Point3f point3d;
	mat D = mat(3, 3, fill::zeros);
	D(0, 0) = lw(0, 0);
	D(0, 1) = lw(0, 1);
	D(0, 2) = lw(0, 2);

	D(1, 0) = lw(1, 0);
	D(1, 1) = lw(1, 1);
	D(1, 2) = lw(1, 2);

	D(2, 0) = lw(2, 0);
	D(2, 1) = lw(2, 1);
	D(2, 2) = lw(2, 2);

	double d_det = det(D);

	mat D1 = mat(3, 3, fill::zeros);
	D1(0, 0) = zc*point2d.x - lw(0, 3); 
	D1(0, 1) = lw(0, 1);
	D1(0, 2) = lw(0, 2);
	
	D1(1, 0) = zc*point2d.y - lw(1, 3);
	D1(1, 1) = lw(1, 1);
	D1(1, 2) = lw(1, 2);

	D1(2, 0) = zc - lw(2, 3);
	D1(2, 1) = lw(2, 1);
	D1(2, 2) = lw(2, 2);

	double d1_det = det(D1);
	point3d.x = d1_det / d_det; 

	mat D2 = mat(3, 3, fill::zeros);
	D2(0, 0) = lw(0, 0);
	D2(0, 1) = zc*point2d.x - lw(0, 3); //
	D2(0, 2) = lw(0, 2);
	

	D2(1, 0) = lw(1, 0);
	D2(1, 1) = zc*point2d.y - lw(1, 3); 
	D2(1, 2) = lw(1, 2);

	D2(2, 0) = lw(2, 0);
	D2(2, 1) = zc - lw(2, 3);
	D2(2, 2) = lw(2, 2);

	double d2_det = det(D2);
	point3d.y = d2_det / d_det;

	mat D3 = mat(3, 3, fill::zeros);
	D3(0, 0) = lw(0, 0);
	D3(0, 1) = lw(0, 1);//
	D3(0, 2) = zc*point2d.x - lw(0, 3); 


	D3(1, 0) = lw(1, 0);
	D3(1, 1) = lw(1, 1);
	D3(1, 2) = zc*point2d.y - lw(1, 3); 

	D3(2, 0) = lw(2, 0);
	D3(2, 1) = lw(2, 1);
	D3(2, 2) = zc - lw(2, 3); 

	double d3_det = det(D3);
	point3d.z = d3_det / d_det;

	return point3d;
	
}

cv::Point2f Camera::Wcs2Ics(cv::Point3f point3d, double disparity)
{
	cv::Point2f p2d;
	double zc = extrinsic(0, 0)*baseline / disparity;

	arma::vec wcs = vec(4);
	wcs(0) = point3d.x;
	wcs(1) = point3d.y;
	wcs(2) = point3d.z;
	wcs(3) = 1;

	mat product = randu<mat>(3, 4);
	product = intrinsic *extrinsic;
	vec result = vec(3);
	result = product * wcs;
	
	p2d.x = result(0) / zc;
	p2d.y = result(1) / zc;
	//point3d.z = result(0) / zc;
	
	return p2d;	
}
void Camera::setRT2NextFrame(std::string matFile)
{
	frameRT.load(matFile);
}