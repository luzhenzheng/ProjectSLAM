#include "dirent.h"
#include <sys\types.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "StereoCalibFile.h"
#include <fstream> 
#include <iomanip>
#define _USE_MATH_DEFINES
#include <math.h>


using namespace std;
using namespace cv;

/*
The program performs a stereo calibration and saves the result in OpenCV-XML format and
as better readable csv-file. If the folder of the images contains a folder named
"Corner Display" images with the detected corners of the chessboard are saved in it. 
*/

//#define _MEAS_ "2017-04-28a_P23_post"

//filepath to the folder where the images are, !!! INSERT PATH HERE !!!
//string imageFilePath = "C:\\Users\\alex_X230\\HESSENBOX-DA\\Austausch_PSR18_G1_Stefan (Stefan Luthardt)\\Daten_Stereokalib\\2017-04-28a_P23_post" ;
string LeftImagePath = "C:\\Users\\alex_X230\\HESSENBOX-DA\\Austausch_PSR18_G1_Stefan (Stefan Luthardt)\\Daten_Stereokalib\\2017-04-28a_P23_post\\image_0\\";
string RightImagePath = "C:\\Users\\alex_X230\\HESSENBOX-DA\\Austausch_PSR18_G1_Stefan (Stefan Luthardt)\\Daten_Stereokalib\\2017-04-28a_P23_post\\image_1\\";

//filepath to the folder where results should be saved and the continental estimates were written to  
string dataFilePath = "C:\\Users\\alex_X230\\HESSENBOX-DA\\Austausch_PSR18_G1_Stefan (Stefan Luthardt)\\Daten_Stereokalib\\2017-04-28a_P23_post" ;
string result_suffix = "";

const float squareSize = 10.0; //size in cm of chessboard corners
Size boardSize(9, 7);			// inner corners of chessboard

//const float squareSize = 10.0; Size boardSize(9, 7);	// large chessboard
//const float squareSize = 3.53; Size boardSize(8, 6);	// medium chessboard
//const float squareSize = 2.99; Size boardSize(8, 6);	// small chessboard

int maxCalibrationIterations = 100;	//maximum number of iterations done by stereo calibration
//bool useSubPixelAccuracy = true;	//refine accuracy of corners

bool showPairs = true; // show the pairs with corners for inspection

//select which values should be used as start values for the stereo calibration
int intrinsicStartValue = 2 ; // 0: Zeros(3,3) 1: continental Guess 2:do own calibration and save 3: load last own calibration

//flags for stereo calibration
int useFlags = 0 +
CV_CALIB_USE_INTRINSIC_GUESS +
//CV_CALIB_FIX_INTRINSIC +
//CV_CALIB_FIX_PRINCIPAL_POINT +
//CV_CALIB_FIX_FOCAL_LENGTH +
//CV_CALIB_FIX_ASPECT_RATIO +
//CV_CALIB_ZERO_TANGENT_DIST +
//CV_CALIB_FIX_K1 +
//CV_CALIB_FIX_K2 +
//CV_CALIB_FIX_K3 +
//CV_CALIB_RATIONAL_MODEL+
0;


void Create3DChessboardCorners(Size boardSize, float squareSize, std::vector<Point3f> &corners);
void calibrateSingleCamera(vector<vector<Point3f> > & chessBoardPoints, vector<vector<Point2f> > &imagePoints, Size &imageSize, Mat &intrinsic, Mat &distortion);
//void writeDataToCSV(Mat intrinsicLeft, Mat intrinsicRight, Mat distortionLeft, Mat distortionRight,
	//Mat stereoCalibrierungIntrinsicLeft, Mat stereoCalibrierungIntrinsicRight, Mat stereoDistortionLeft, Mat stereoDistortionRight,
	//Mat translation, Mat rotation, double rmse);


int main() {

	DIR *L_directory = opendir(LeftImagePath.c_str());
	DIR *R_directory = opendir(RightImagePath.c_str());
	//DIR *S_directory = opendir(imageFilePath.c_str());
	struct dirent *Lbild;
	struct dirent *Rbild;
	struct stat filestat;
	//check if directory is valid
	if (L_directory == NULL) {
		cout << "Error(" << errno << ") opening " << L_directory << endl;
	    system("pause");
		return errno;
	}
	else if (R_directory == NULL) {
		cout << "Error(" << errno << ") opening " << R_directory << endl;
		system("pause");
		return errno;
	}

	vector<Mat> rightImages;
	vector<Mat> leftImages;
	vector<string> RNames;
	vector<string> LNames;


	// finds all images in a folder and adds them to the vector images
	// code for getting all files in the directory based on http://www.cplusplus.com/forum/beginner/10292/
	cout<<"Load images...\n"<<endl;
	//namedWindow("Debug1", WINDOW_AUTOSIZE | WINDOW_KEEPRATIO);
	//namedWindow("Debug2", WINDOW_AUTOSIZE | WINDOW_KEEPRATIO);
	Ptr<CLAHE> clahe = cv::createCLAHE(1.2, Size(30, 30));
	while (Lbild = readdir(L_directory)) {
		Mat image;
		string l;
		
		string imName = Lbild->d_name;
		l.append(LeftImagePath);
		l.append(imName);
		image = imread(l, CV_LOAD_IMAGE_GRAYSCALE);


		// If the file is a directory (or is in some way invalid) we'll skip it 
		if (stat(l.c_str(), &filestat)) continue;
		if (S_ISDIR(filestat.st_mode))  continue;

	
		// apply CLAHE
		//imshow("Debug1", image); waitKey(1);
		//clahe->apply(image, image);
		//imshow("Debug2", image); waitKey(0);
		
		//check for correct naming convention
		//if (imName.at(imName.size() - 5) != 'L' && imName.at(imName.size() - 5) != 'R') {
			//cout << "Error:  image cannot be assigned";
			//system("pause");
		//	return -1;
		//}

		//determine if left or right image
		//else if (imName.at(imName.size() - 5) == 'L') {
			leftImages.push_back(image);
			LNames.push_back(imName);
		//}
		//else {
		//	rightImages.push_back(image);
		//	RNames.push_back(imName);
		//}
	}
	closedir(L_directory);
	while (Rbild = readdir(R_directory)) {
		Mat image;
		string r;
		r.append(RightImagePath);
		string imName = Rbild->d_name;
		r.append(imName);
		//cout <<r<< endl;
		if (stat(r.c_str(), &filestat)) continue;
		if (S_ISDIR(filestat.st_mode))  continue;
		image = imread(r, CV_LOAD_IMAGE_GRAYSCALE);
		//imshow("Debug1", image); waitKey(1);
		rightImages.push_back(image);
		RNames.push_back(imName);
	}
	closedir(R_directory);


	// use the collected images to perform a stereo calibration


	// find corners in all images and add corners to vectors if the
	// chessboard is recognized in the left and right image
	vector<vector<Point2f> > leftCameraImagePoints;
	vector<vector<Point2f> > rightCameraImagePoints;

	cout << "Find corners...\n";
	// setup
	if (showPairs)  namedWindow("Show", WINDOW_AUTOSIZE | WINDOW_KEEPRATIO);
	//Size subpix_winSize = Size(8, 8);
	//Size subpix_zeroZone = Size(-1, -1);
	//TermCriteria subpix_criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.001);
	Size region_size = Size(11, 11);
	int writeCounter = 0;

	for (int i = 0; i < leftImages.size(); i++) {
		//if folder "Corner Display" exists in the image folder, save images with marked corners 
		stringstream left;
		stringstream right;
		left << LeftImagePath << "Corner Display\\img_" << writeCounter << "_L.jpg";
		right << RightImagePath << "Corner Display\\img_" << writeCounter << "_R.jpg";
		Mat outImgL;
		vector<Point2f> leftPoints;
		//outImgL = leftImages[i].clone();
		//cvtColor(leftImages[i], leftImages[i], cv::COLOR_RGB2GRAY);
		bool foundLeft = findChessboardCorners(leftImages[i], boardSize, leftPoints, CALIB_CB_ADAPTIVE_THRESH);
		//if (0 == foundLeft)
			//cout << "No corner found.";
		if(foundLeft) find4QuadCornerSubpix(leftImages[i], leftPoints, region_size);
		
		cvtColor(leftImages[i], outImgL, cv::COLOR_GRAY2RGB);
		drawChessboardCorners(outImgL, boardSize, leftPoints, foundLeft);
		imwrite(left.str(), outImgL);
		
		
		Mat outImgR;
		vector<Point2f> rightPoints;
		//outImgR = rightImages[i].clone();
		//cvtColor(rightImages[i], rightImages[i], cv::COLOR_RGB2GRAY);
		bool foundRight = findChessboardCorners(rightImages[i], boardSize, rightPoints, CALIB_CB_ADAPTIVE_THRESH);
		//if (foundRight)  cornerSubPix(rightImages[i], rightPoints, subpix_winSize, subpix_zeroZone, subpix_criteria);
		if (foundRight)  find4QuadCornerSubpix(rightImages[i], rightPoints, region_size);
		cvtColor(rightImages[i], outImgR, cv::COLOR_GRAY2RGB);
		drawChessboardCorners(outImgR, boardSize, rightPoints, foundRight);
		imwrite(right.str(), outImgR);
		writeCounter++;

		if (foundLeft && foundRight) {
			leftCameraImagePoints.push_back(leftPoints);
			rightCameraImagePoints.push_back(rightPoints);
		} else {
			if (!foundLeft)  std::cerr << "Checkerboard corners not found! left ImageNr: " << i << " (" + LNames.at(i) + ")" << std::endl;
			if (!foundRight)  std::cerr << "Checkerboard corners not found! right ImageNr: " << i << " (" + RNames.at(i) + ")" << std::endl;
		}
		// show images
		if (showPairs) {
			Mat disp;
			hconcat(outImgL, outImgR, disp);
			resize(disp, disp, Size(), 0.75, 0.75);
			cv::putText(disp, LNames.at(i).c_str(), cvPoint(10, 30), FONT_HERSHEY_SIMPLEX, 1, cvScalar(0, 0, 255), 2);
			imshow("Show", disp); waitKey(1);
			//destroyWindow("Show");
	}
	}
	if(showPairs)  destroyWindow("Show");
	

	//setup chessboard struct
	vector<vector<Point3f> > objectPoints(leftCameraImagePoints.size());
	vector<Point3f>  chessboardPoints;
	Create3DChessboardCorners(boardSize, squareSize, chessboardPoints);
	for (int i = 0; i < leftCameraImagePoints.size(); i++) {
		objectPoints[i] = chessboardPoints;
	}

	Mat cameraMatrix[2];
	Mat distortionCoefficients[2];
	Size imageSize = leftImages[1].size();
	// load start values according to selected parameters

	//get file path to own calibration matrices
	string ownCalibLeft;
	ownCalibLeft.append(dataFilePath);
	ownCalibLeft.append("OwnCalibMatLeft.xml");

	string ownCalibRight;
	ownCalibRight.append(dataFilePath);
	ownCalibRight.append("OwnCalibMatRight.xml");

	if (intrinsicStartValue == 0) {
		cameraMatrix[0] = Mat::zeros(3, 3, CV_32F);
		cameraMatrix[1] = Mat::zeros(3, 3, CV_32F);
		distortionCoefficients[0] = Mat::zeros(1, 5, CV_32F);
		distortionCoefficients[1] = Mat::zeros(1, 5, CV_32F);
	}
	else if (intrinsicStartValue == 1) {
		//read in continental guess
		string continentalLeft;
		continentalLeft.append(dataFilePath);
		continentalLeft.append("CalibMatLeft.xml");

		string continentalRight;
		continentalRight.append(dataFilePath);
		continentalRight.append("CalibMatRight.xml");
		FileStorage leftRead(continentalLeft, FileStorage::READ);
		FileStorage rightRead(continentalRight, FileStorage::READ);

		leftRead["cameraMatrix"] >> cameraMatrix[0];
		rightRead["cameraMatrix"] >> cameraMatrix[1];
		leftRead["distortion"] >> distortionCoefficients[0];
		rightRead["distortion"] >> distortionCoefficients[1];
	}
	else if (intrinsicStartValue == 2) {
		cout << "Mono calibration...\n";
		//perform mono calibration
		calibrateSingleCamera(objectPoints, leftCameraImagePoints, imageSize, cameraMatrix[0], distortionCoefficients[0]);
		calibrateSingleCamera(objectPoints, rightCameraImagePoints, imageSize, cameraMatrix[1], distortionCoefficients[1]);
		// output results on console
		std::cout << "Mono Calibration-----------------------------------------------\n";
		std::cout << "left Camera: \n";
		for (int i = 0; i < 2; i++) {
			std::cout << "Camera Matrix: \n" << cameraMatrix[i] << endl;
			std::cout << "Distortion Matrix: \n" << distortionCoefficients[i] << endl;
			std::cout << "right Camera \n";
		}
		//store results of mono calibration
		FileStorage fsLeft(ownCalibLeft, FileStorage::WRITE);
		FileStorage fsRight(ownCalibRight, FileStorage::WRITE);
		fsLeft << "cameraMatrix" << cameraMatrix[0] << "distortion" << distortionCoefficients[0];
		fsRight << "cameraMatrix" << cameraMatrix[1] << "distortion" << distortionCoefficients[1];
		fsLeft.release();
		fsRight.release();
	}
	else {
		//read last mono calibration
		FileStorage leftRead(ownCalibLeft, FileStorage::READ);
		FileStorage rightRead(ownCalibRight, FileStorage::READ);

		leftRead["cameraMatrix"] >> cameraMatrix[0];
		rightRead["cameraMatrix"] >> cameraMatrix[1];
		leftRead["distortion"] >> distortionCoefficients[0];
		rightRead["distortion"] >> distortionCoefficients[1];
	}
	double total_error = 0.0;
	double error = 0.0;
	for (int i = 0; leftImages.size(); i++) {
		vector<Point3f> tempPointSet = objectPoints[i];
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
		fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	std::cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
	std::cout << "评价完成！" << endl;
	}
	vector<Point2f> leftCameraImagePoints2;
	vector<Point2f> rightCameraImagePoints2;

	Mat intrinsicLeftBefore = cameraMatrix[0].clone();
	Mat intrinsicRightBefore = cameraMatrix[1].clone();
	Mat distortionLeftBefore = distortionCoefficients[0].clone();
	Mat distortionRightBefore = distortionCoefficients[1].clone();

	Mat rotationMatrix;
	Mat translationVector;
	Mat essentialMatrix;
	Mat fundamentalMatrix;
	//set termination criteria
	TermCriteria stereoTermCriteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, maxCalibrationIterations, 1e-6);

	// perform stereo calibration like in https://github.com/opencv/opencv/blob/master/samples/cpp/stereo_calib.cpp
	cout << "Stereo calibration...\n";
	double rms = stereoCalibrate(objectPoints, leftCameraImagePoints, rightCameraImagePoints,
		cameraMatrix[0], distortionCoefficients[0],
		cameraMatrix[1], distortionCoefficients[1],
		imageSize, rotationMatrix, translationVector, essentialMatrix, fundamentalMatrix,
		useFlags,
		stereoTermCriteria);

	// --- SAVE CALIBRATION RESULTS -------------------------------------------
	cout << "Save results...\n";
	string stereocalibResultPath;
	stereocalibResultPath.append(dataFilePath);
	stereocalibResultPath.append("StereoCalibration");
	stereocalibResultPath.append(result_suffix);
	stereocalibResultPath.append(".xml");
	
	// we need L2R transformation
	Mat R_L2R = rotationMatrix.t(); // for some reason R has to be "inverted"
	Mat t_L2R = translationVector;
	writeStereoCalib(stereocalibResultPath, imageSize, cameraMatrix[0], distortionCoefficients[0],
		cameraMatrix[1], distortionCoefficients[1], R_L2R, t_L2R, rms, CALIB_UNIT_CM);

	
	// --- Save calibration points (for statistics) ---------------------------
	stereocalibResultPath.clear();
	stereocalibResultPath.append(dataFilePath);
	stereocalibResultPath.append("points.txt");
	std::ofstream pointfile(stereocalibResultPath);
	for (int i = 0; i < leftCameraImagePoints.size(); i++) {
		for (int j = 0; j < leftCameraImagePoints[i].size(); j++) {
			Point2f point = leftCameraImagePoints[i][j];
			pointfile << fixed << setprecision(2) << point.x << "; " << fixed << setprecision(2) << point.y << "; ";
		}
		pointfile << '\n';
	}
	pointfile.close();


	// --- SAVE CALIBRATION RESULTS (legacy code, format from group 4) --------

	//xml storage in opencv xml format, can used to reload data later e.g. in continental framework
	stereocalibResultPath.clear();
	stereocalibResultPath.append(dataFilePath);
	stereocalibResultPath.append("CalibrationResult_G4.xml");
	FileStorage fs(stereocalibResultPath, FileStorage::WRITE);
	fs << "IntrinsicMatLeft" << cameraMatrix[0];
	fs << "distCoefficientsLeft" << distortionCoefficients[0];
	fs << "IntrinsicMatRight" << cameraMatrix[1];
	fs << "distCoefficientsRight" << distortionCoefficients[1];
	fs << "translationVector" << translationVector;
	fs << "rotationMatrix" << rotationMatrix;
	fs << "RMSE" << rms;
	fs.release();
	
	// print data on console
	std::cout << "Stereo Calibration-----------------------------------------------\n";
	std::cout << "done with RMS error=" << rms << std::endl;
	std::cout << "left Camera: \n";
	for (int i = 0; i < 2; i++) {
		std::cout << "Camera Matrix: \n" << cameraMatrix[i] << endl;
		std::cout << "Distortion Matrix: \n" << distortionCoefficients[i] << endl;
		std::cout << "right Camera \n";
	}
	std::cout << "Translation: \n" << translationVector << endl;
	std::cout << "Rotation: \n" << rotationMatrix << endl;
	
	//write Data to CSV-File
	writeDataToCSV(intrinsicLeftBefore, intrinsicRightBefore, distortionLeftBefore, distortionRightBefore, cameraMatrix[0], cameraMatrix[1], distortionCoefficients[0], distortionCoefficients[1], translationVector, rotationMatrix, rms);

	cout << "Done.\n";
	system("pause");
	return 0;
}

//conversion of rotational matrix into Euler angles
//code from https://www.learnopencv.com/rotation-matrix-to-euler-angles/
Point3f rotMatToEuler(Mat & R) {
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = sy < 1e-6;

	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	//conversion to degree
	x = x / M_PI * 180;
	y = y / M_PI * 180;
	z = z / M_PI * 180;
	return Point3f(x, y, z);
}

//write the data into a csv-file
void writeDataToCSV(Mat intrinsicLeft, Mat intrinsicRight, Mat distortionLeft, Mat distortionRight,
	Mat stereoCalibrierungIntrinsicLeft, Mat stereoCalibrierungIntrinsicRight, Mat stereoDistortionLeft, Mat stereoDistortionRight,
	Mat translation, Mat rotation, double rmse) {
	string csvPath;
	csvPath.append(dataFilePath);
	csvPath.append("Results.csv");
	std::ofstream outfile(csvPath, std::ofstream::binary);
	outfile.imbue(locale("german"));
	outfile << ";Intrinsich Links Vor kalibrierung;Intrinsich Links nach Kalibrierung;Intrinsich Recht Vor kalibrierung;Intrinsich Rechts nach Kalibrierung\n";
	outfile << "Fx;" << intrinsicLeft.at<double>(0, 0) << ";" << stereoCalibrierungIntrinsicLeft.at<double>(0, 0) << ";" << intrinsicRight.at<double>(0, 0) << ";" << stereoCalibrierungIntrinsicRight.at<double>(0, 0) << endl;
	outfile << "Fy;" << intrinsicLeft.at<double>(1, 1) << ";" << stereoCalibrierungIntrinsicLeft.at<double>(1, 1) << ";" << intrinsicRight.at<double>(1, 1) << ";" << stereoCalibrierungIntrinsicRight.at<double>(1, 1) << endl;
	outfile << "Px;" << intrinsicLeft.at<double>(0, 2) << ";" << stereoCalibrierungIntrinsicLeft.at<double>(0, 2) << ";" << intrinsicRight.at<double>(0, 2) << ";" << stereoCalibrierungIntrinsicRight.at<double>(0, 2) << endl;
	outfile << "Py;" << intrinsicLeft.at<double>(1, 2) << ";" << stereoCalibrierungIntrinsicLeft.at<double>(1, 2) << ";" << intrinsicRight.at<double>(1, 2) << ";" << stereoCalibrierungIntrinsicRight.at<double>(1, 2) << endl;
	outfile << "K1;" << distortionLeft.at<double>(0) << ";" << stereoDistortionLeft.at<double>(0) << ";" << distortionRight.at<double>(0) << ";" << stereoDistortionRight.at<double>(0) << endl;
	outfile << "K2;" << distortionLeft.at<double>(1) << ";" << stereoDistortionLeft.at<double>(1) << ";" << distortionRight.at<double>(1) << ";" << stereoDistortionRight.at<double>(1) << endl;
	outfile << "K3;" << distortionLeft.at<double>(4) << ";" << stereoDistortionLeft.at<double>(4) << ";" << distortionRight.at<double>(4) << ";" << stereoDistortionRight.at<double>(4) << endl;
	outfile << "P1;" << distortionLeft.at<double>(2) << ";" << stereoDistortionLeft.at<double>(2) << ";" << distortionRight.at<double>(2) << ";" << stereoDistortionRight.at<double>(2) << endl;
	outfile << "P2;" << distortionLeft.at<double>(3) << ";" << stereoDistortionLeft.at<double>(3) << ";" << distortionRight.at<double>(3) << ";" << stereoDistortionRight.at<double>(3) << endl;

	outfile << "Stereo Kalibrierung" << endl;
	outfile << "tx;" << translation.at<double>(0) << endl;
	outfile << "ty;" << translation.at<double>(1) << endl;
	outfile << "tz;" << translation.at<double>(2) << endl;

	Point3f euler = rotMatToEuler(rotation);
	outfile << "rotx in grad;" << euler.x << endl;
	outfile << "roty in grad;" << euler.y << endl;
	outfile << "rotz in grad;" << euler.z << endl;

	outfile << "RMSE Stereo;" << rmse << endl;
	outfile.close();
}

//performs mono calibration of a single camera
void calibrateSingleCamera(vector<vector<Point3f> > & chessBoardPoints, vector<vector<Point2f> > &imagePoints, Size &imageSize, Mat &intrinsic, Mat &distortion) {
	Mat rot;
	Mat trans;
	calibrateCamera(chessBoardPoints, imagePoints, imageSize, intrinsic, distortion, rot, trans);
}

//creates a vector of points describing the chessboard
void Create3DChessboardCorners(Size boardSize, float squareSize, std::vector<Point3f> &corners) {
	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(Point3f(float(j*squareSize),
				float(i*squareSize), 0));
		}
	}
}

