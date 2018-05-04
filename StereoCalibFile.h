#pragma once
#include <string>
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

const int CALIB_UNIT_MM = 1000;
const int CALIB_UNIT_CM = 100;
const int CALIB_UNIT_M = 1;


void writeStereoCalib(string filename, Size imgSize, Mat leftK, Mat leftDistortion,
					  Mat rightK, Mat rightDistortion, Mat R_L2R, Mat t_L2R,
					  double rmse = 0.0, int unit = CALIB_UNIT_M);

void readStereoCalib(string filename, Size &imgSize, Mat &leftK, Mat &leftDistortion,
					 Mat &rightK, Mat &rightDistortion, Mat &R_L2R, Mat &t_L2R,
					 int unit = CALIB_UNIT_M);
