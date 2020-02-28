#include <iostream>
#include <string>
#include <opencv2\opencv.hpp>

#include "LaneDetection.h"

void main(){
	
	// input parameter
	bool verbose_lm_detction = true;
	bool verbose_seed_gen = true;
	bool verbose_run_crf = true;
	bool verbose_validating = true;
	bool verbose = verbose_lm_detction | verbose_seed_gen | verbose_run_crf | verbose_validating;

	//<-------------- Common Variables definition & initialization --------------> 
	std::string img_path = "..\\data\\img_mask\\resized\\";
	//\60\road_images_out

	LaneDetection ld = LaneDetection();
	std::string img_name = (std::string(img_path)).append(std::to_string(1)).append(".jpg");
	
	// initilaize
  	if (!ld.initialize_variable(img_name)) {
  		return;
  	}
	
	// process
	for(int ff=60;ff<73;ff++){

		std::string img_name = (std::string(img_path)).append(std::to_string(ff)).append(".jpg");		

		// Initlaization
		if (!ld.initialize_Img(img_name)) {
			continue;
		}
		
		// detecting lane markings
		ld.lane_marking_detection(verbose_lm_detction);

		// supermarking generation and low-level association
		ld.seed_generation(verbose_seed_gen);

		// CRF graph configuration & optimization using hungarian method
		ld.graph_generation(verbose_run_crf);
		
		// validating
		ld.validating_final_seeds(verbose_validating);
	

		if (verbose) {
			cv::waitKey(0);
		}
	}

	ld.~LaneDetection();
	
}