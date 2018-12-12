#include "pch.h"
#include "VideoDecoder.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

/*
* Initialize VideoDecoder. It will init the default parameters.
* in: 
*	- Size - the size of intersections' corrdinate system
*	- float - measured width of a chessboard square
*/
VideoDecoder::VideoDecoder(Size chessboardIntersections, float chessboardSqueareDimension)
{
	Mat rVec = Mat::zeros(3, 1, CV_64FC1);
	Mat tVec = Mat::zeros(3, 1, CV_64FC1);
	capture = new VideoCapture(1);

	chessboardDimensions = chessboardIntersections;
	calibrationSquareDimension = chessboardSqueareDimension;

	buildCamCalibMatrices();
	createKnownChessboardPoints();
}


VideoDecoder::~VideoDecoder()
{
}

/* Method for building the camera and disance coeff matrices */
void VideoDecoder::buildCamCalibMatrices() {
	camMatrix = Mat(3, 3, CV_64F);

	camMatrix.at<double>(0, 0) = 1601.3;
	camMatrix.at<double>(0, 1) = 0;
	camMatrix.at<double>(0, 2) = -9.00041;

	camMatrix.at<double>(1, 0) = 0;
	camMatrix.at<double>(1, 1) = 2724.92;
	camMatrix.at<double>(1, 2) = 8.27411;

	camMatrix.at<double>(2, 0) = 0;
	camMatrix.at<double>(2, 1) = 0;
	camMatrix.at<double>(2, 2) = 1;

	distCoefMatrix = Mat(8, 1, CV_64F);

	distCoefMatrix.at<double>(0, 0) = -0.321751;
	distCoefMatrix.at<double>(1, 0) = 4.55161;
	distCoefMatrix.at<double>(2, 0) = -0.0737929;
	distCoefMatrix.at<double>(3, 0) = -0.170011f;

	distCoefMatrix.at<double>(4, 0) = -9.47005;
	distCoefMatrix.at<double>(5, 0) = 0;
	distCoefMatrix.at<double>(6, 0) = 0;
	distCoefMatrix.at<double>(7, 0) = 0;

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

	if (!capturedFrame.empty())
	{
		cvtColor(capturedFrame, grayFrame, CV_BGR2GRAY);
		foundChessboard = findChessboardCorners(grayFrame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

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
		}
	}
		
	return foundChessboard;
}
