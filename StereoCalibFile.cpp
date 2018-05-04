#include "StereoCalibFile.h"

// write stereo calibration data to XML-File
void writeStereoCalib(string filename, Size imgSize, Mat leftK, Mat leftDistortion,
					  Mat rightK, Mat rightDistortion, Mat R_L2R, Mat t_L2R, double rmse, int unit) {
	
	// unit conversion (always stored in meters)
	Mat t_L2R_conv;
	t_L2R_conv = t_L2R/(double)unit;

	// store in OpenCV-Filestorage
	FileStorage calibFile(filename, FileStorage::WRITE);
	calibFile << "ImageSize" << imgSize;	
	calibFile << "LeftCameraMatrix" << leftK;
	calibFile << "LeftDistortion" << leftDistortion;	
	calibFile << "RightCameraMatrix" << rightK;
	calibFile << "RightDistortion" << rightDistortion;
	calibFile << "Rotation_Left2Right" << R_L2R;
	calibFile << "Translation_Left2Right" << t_L2R_conv;
	calibFile << "Calibration_RMSE" << rmse;
	calibFile.release();
}


// read stereo calibration data from XML-File
void readStereoCalib(string filename, Size &imgSize, Mat &leftK, Mat &leftDistortion,
					 Mat &rightK, Mat &rightDistortion, Mat &R_L2R, Mat &t_L2R, int unit) {
	
	// load from OpenCV-Filestorage
	FileStorage calibFile(filename, FileStorage::READ);
	calibFile["ImageSize"] >> imgSize;
	calibFile["LeftCameraMatrix"] >> leftK;
	calibFile["LeftDistortion"] >> leftDistortion;
	calibFile["RightCameraMatrix"] >> rightK;
	calibFile["RightDistortion"] >> rightDistortion;
	calibFile["Rotation_Left2Right"] >> R_L2R;
	calibFile["Translation_Left2Right"] >> t_L2R;
	// (rmse not needed)
	calibFile.release();

	// unit conversion (always stored in meters)
	t_L2R = t_L2R*(double)unit;
}
