#include "pch.h"
#include "VideoDecoder.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <fstream>
#include <assert.h>
using namespace std;
using namespace cv;

/*
* Initialize VideoDecoder. It will init the default parameters.
* in: 
*	- Size - the size of intersections' corrdinate system
*	- float - measured width of a chessboard square
*/
VideoDecoder::VideoDecoder(bool readFromFile, string pathToCalibResult, Size chessboardIntersections, float chessboardSqueareDimension, string pathToVideoFile) {
	Mat rVec = Mat::zeros(3, 1, CV_64FC1);
	Mat tVec = Mat::zeros(3, 1, CV_64FC1);

	if (readFromFile) {		
		capture = new VideoCapture(pathToVideoFile);
	}
	else {
		capture = new VideoCapture(1);
	}

	//assert(capture->isOpened() && "video file or webcam stream cannot be opened");

	chessboardDimensions = chessboardIntersections;
	calibrationSquareDimension = chessboardSqueareDimension;

	buildCamCalibMatrices(pathToCalibResult);
	createKnownChessboardPoints();
}


VideoDecoder::~VideoDecoder()
{
}

/* Method for building the camera and disance coeff matrices from given file, where the calibration parametrer has been stored by the calib app */
void VideoDecoder::buildCamCalibMatrices(string calibResultsPath) {
	
	ifstream inputStream(calibResultsPath);
	string line;
	vector<string> lines;

	// open - load - close
	while (getline(inputStream, line)){
		lines.push_back(line);
	}
	inputStream.close();

	//3x3 + 5 parameter
	assert(lines.size() == 14  && "there is no enough parameter in the calibration file" );

	int lineNum = 0;

	// build up the matrices and show them on the console
	cout << "camera matrix:" << endl;
	camMatrix = Mat(3, 3, CV_64F);
	for (int rowNum = 0; rowNum < camMatrix.rows; rowNum++) {
		for (int colNum = 0; colNum < camMatrix.cols; colNum++) {
			camMatrix.at<double>(rowNum, colNum) = stod(lines[lineNum]);			
			lineNum++;

			cout << "  " << camMatrix.at<double>(rowNum, colNum);
		}
		cout << endl;
	}
	
	cout << "distance coefficients: " << endl;
	distCoefMatrix = Mat(5, 1, CV_64F);
	for (int rowNum = 0; rowNum < distCoefMatrix.rows; rowNum++) {
		distCoefMatrix.at<double>(rowNum, 0) = stod(lines[lineNum]);
		lineNum++;

		cout << "  " << distCoefMatrix.at<double>(rowNum, 0) << endl;
	}
}

/* Method for calculating the coordinates of the real chessboard */
void VideoDecoder::createKnownChessboardPoints() {
	for (int i = 0; i < chessboardDimensions.height; i++)
	{
		for (int j = 0; j < chessboardDimensions.width; j++)
		{
			modelPoints.push_back(Point3f(j*calibrationSquareDimension, i*calibrationSquareDimension, 0.0f));
		}
	}
}

/* Method for capturing the next valid frame. Returns with the success of the capture */
bool VideoDecoder::nextFrame() {
	waitKey(1); // not so harsh, the camera cannot handle too many requests --> the capture should be slowed 
	return capture->read(capturedFrame);
		
}
/*
*  Method for decoding the image. 
*	0. check that the frame is empty or not. If not:
*	1. (builtin) convert the frame to grayscale. It makes the following algos job easier
*	2. (builtin) find the image coordinates of the chessboard (if there is one) 
*	3. (builtin) solve the association problem between the image points and the real points with PnPRansac algorithm --> it gives back the translation and rotation vectors
*	4. yeeey :-)
*/
bool VideoDecoder::decodeFrame() {
	bool foundChessboard = false;
	vector<Point2f> foundPoints;

	if (!capturedFrame.empty()) {		
		cvtColor(capturedFrame, grayFrame, CV_BGR2GRAY);
		foundChessboard = findChessboardCorners(capturedFrame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
		//drawChessboardCorners(capturedFrame, chessboardDimensions, foundPoints, foundChessboard);

		if (foundChessboard) {
			/* for drawing
			TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001);
			Size zerozone = Size(-1, 1);
			Size winSize = Size(7, 7);
			cornerSubPix(grayFrame, foundPoints, winSize, zerozone, criteria);
			*/
			
			// re-init the vectors
			rVec = Mat::zeros(3, 1, CV_64FC1);
			tVec = Mat::zeros(3, 1, CV_64FC1);
			solvePnPRansac(modelPoints, foundPoints, camMatrix, distCoefMatrix, rVec, tVec);
			//solvePnP(modelPoints, foundPoints, camMatrix, distCoefMatrix, rVec, tVec);
		}
	}
		
	return foundChessboard;
}

void VideoDecoder::release() {
	capture->release();
}


void VideoDecoder::readAllFrames() {

	cout << "capture all frames ..";
	while (true) {
		Mat frame;
		if (!capture->read(frame))
			break;
		if (frame.empty())
			break;

		capturedFrames.push_back(frame);
		waitKey(1);
		cout << ".";
		
	}
	cout << "captured frames: " << capturedFrames.size() << endl;
}

bool VideoDecoder::decodeGivenFrame(Mat frame) {
	capturedFrame = frame;

	return decodeFrame();
}