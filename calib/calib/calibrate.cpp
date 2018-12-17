
#include "pch.h"
#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "calibrate.hpp"

#include <sstream>
#include <fstream>
using namespace cv;
using namespace std;



CameraCalibrator::CameraCalibrator(bool readFromFile, string inputFileName, string outputFileName, Size intersectionNum, float chessSquareDim) {
	outFilePath = outputFileName;

	//chessboardDimensions = Size(7, 7);
	chessboardDimensions = intersectionNum;
	calibrationSquareDimension = chessSquareDim;


	if (readFromFile) {		
		videoCapture = new VideoCapture(inputFileName);
	}
	else {
		videoCapture = new VideoCapture(0);
	}
	cameraMatrix = Mat::eye(3, 3, CV_64F);		

	createKnownBoardPositions();
}

CameraCalibrator::~CameraCalibrator() {

}

void CameraCalibrator::createKnownBoardPositions() {
	vector<Point3f> corners; // = worldSpaceCornerPoints[0];
	
	for (int i = 0; i < chessboardDimensions.height; i++)
	{
		for (int j = 0; j < chessboardDimensions.width; j++)
		{
			corners.push_back(Point3f(j*calibrationSquareDimension, i*calibrationSquareDimension, 0.0f));			
		}
	}
	cout << "num of corners: " << corners.size() << endl;
	
	worldSpaceCornerPoints.push_back(corners);
}

bool CameraCalibrator::saveCameraCalibration() {
	ofstream outStream(outFilePath);

	if (outStream)
	{
		uint16_t rows = cameraMatrix.rows;
		uint16_t columns = cameraMatrix.cols;

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double value = cameraMatrix.at<double>(r, c);
				outStream << value << endl;
			}
		}

		rows = distanceCoefficients.rows;
		columns = distanceCoefficients.cols;

		cout << "distCoeff rows: " << rows << endl;
		cout << "distCoeff columns: " << columns << endl;


		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double value = distanceCoefficients.at<double>(r, c);
				outStream << value << endl;
			}
		}

		outStream.close();
		cout << "calibration data saved" << endl;
		return true;
	}

	return false;
}

void CameraCalibrator::performCalibration(int skip) {
	
	int skipped = 0;
	while (true)
	{
		if (!videoCapture->read(frame))
		{
			cout << "capture was not successfull. The program terminates." << endl;
			break;
		}
		if (skipped < skip) {
			skipped++;
		}
		else {
			skipped = 0;
			bool found = false;

			found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE); // | CV_CALIB_CB_FAST_CHECK);
			frame.copyTo(drawToFrame);

			drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);

			if (found)
			{
				imshow("Webcam", drawToFrame);
				savedImages.push_back(drawToFrame);
				checkerboardImageSpacePoints.push_back(foundPoints);

				cout << "saved images: " << savedImages.size() << endl;
				if (checkerboardImageSpacePoints.size() > 30)
				{
					worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);
					vector<Point3f> oneReal = worldSpaceCornerPoints[0];
					cout << "real corners cols: " << worldSpaceCornerPoints[0].size() << endl;
					cout << "worldSpaceCornerPoints.size: " << worldSpaceCornerPoints.size() << endl;
					vector<Mat> rVectors, tVectors;
					distanceCoefficients = Mat::zeros(8, 1, CV_64F);

					cout << "calibrating camera ...";
					calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, chessboardDimensions, cameraMatrix, distanceCoefficients, rVectors, tVectors);
					cout << " [OK]" << endl;

					saveCameraCalibration();
					cout << "rVectors.size: " << rVectors.size() << endl;
					cout << "tVectors.size: " << tVectors.size() << endl;
					break;
				}
			}
			else
			{
				imshow("Webcam", frame);
				cout << "not found" << endl;
			}
		}
	
		char character = waitKey(1);

	}
}
