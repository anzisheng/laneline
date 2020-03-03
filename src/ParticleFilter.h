#pragma  once
#include <iostream>
#include <string>
#include<iostream>
#include<fstream>
#include <armadillo>

#include <opencv2\opencv.hpp>
#define  PARTICLE_NUM 200

class ParticleFilter
{

	ParticleFilter();
public:
	std::vector<cv::Point2f> m_pts;
	std::vector<float> m_coeff(3);
	int n_particle_;
};