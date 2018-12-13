#include "pch.h"
#include "PositionEstimator.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

PositionEstimator::PositionEstimator()
{
	KF = KalmanFilter(4, 2, 0);
	KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
	//Mat_<float> measurement_(2, 1);
	//measurement = measurement_;
	measurement.create(2, 1);
	measurement.setTo(Scalar(0));

	KF.statePre.at<float>(0) = 0.0; //relpos.x at the beginning
	KF.statePre.at<float>(1) = 0.0; // relpos.y at the beginning
	KF.statePre.at<float>(2) = 0.0;
	KF.statePre.at<float>(3) = 0.0;

	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(10));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
	kalmanv.clear();
	
	
}


PositionEstimator::~PositionEstimator()
{
}

void PositionEstimator::addNewPoseVectors(Mat translationVector, Mat rotationVector) {
	trVec = translationVector;
	rotVec = rotationVector;
}

void PositionEstimator::calculatePosition() {
	
	double camFps = 50;
	double frameTime = 1 / camFps;
	Coordinates relVel;
	Coordinates relPos;
	Coordinates relPosLast;
	Coordinates posCurr;
	Coordinates translateVectCurr;
	Coordinates filteredRelPos;

	vector<Coordinates> diffVect;

	// convert to vehicle coordinate sys
	translateVectCurr.x = trVec.at<double>(2, 0);
	translateVectCurr.y = trVec.at<double>(0, 0);
	
	pastTranslateVects.push_back(translateVectCurr);	

	if (pastTranslateVects.size() > 2) {

		relPos.x = translateVectCurr.x - translateVectOrig.x;
		relPos.y = translateVectCurr.y - translateVectOrig.y;
		relPos.Sum = sqrt(relPos.x * relPos.x + relPos.y *relPos.y);

		relPosLast = pastRelPosVects.back();
		cout << frameTime << endl;
		relVel.x = (relPos.x - relPosLast.x) / frameTime;
		relVel.y = (relPos.y - relPosLast.y) / frameTime;
		relVel.Sum = sqrt(relVel.x * relVel.x + relVel.y * relVel.y);


		//cout << " r: " << relPos.Sum;
		//cout << " rx: " << relPos.x;
		//cout << " ry: " << relPos.y;
		//cout << " | v " << relVel.Sum;
		//cout << " vx: " << relVel.x;
		//cout << " vy: " << relVel.y << endl;

		
		Mat prediction = KF.predict();
		Point predictPoint(prediction.at<float>(0), prediction.at<float>(1));

		// measure
		measurement(0) = relPos.x;
		measurement(1) = relPos.y;

		// update
		Mat estimated = KF.correct(measurement);

		Point statePoint(estimated.at<float>(0), estimated.at<float>(1));
		filteredRelPos.x = estimated.at<float>(0);
		filteredRelPos.y = estimated.at<float>(1);
		filteredRelPos.Sum = sqrt(filteredRelPos.x * filteredRelPos.x + filteredRelPos.y * filteredRelPos.y);
		filteredPastRelPosVects.push_back(filteredRelPos);

		cout << " x: " << filteredRelPos.x;
		cout << " y: " << filteredRelPos.y << endl;

		showGraph();

	}
	else {
		translateVectOrig.x = translateVectCurr.x;
		translateVectOrig.y = translateVectCurr.y;

		relPos.x = 0;
		relPos.y = 0;
		relPos.Sum = 0;		
	}
	pastRelPosVects.push_back(relPos);


}

void PositionEstimator::showGraph() {
	int windowSizeX = 512;
	int windowSizeY = 512;
	int scale = 500;
	int posX, posY, posXbefore, posYbefore;

	namedWindow("graph");
	vector<Coordinates> posDiff;
	Mat img = Mat::zeros(windowSizeX, windowSizeY, CV_8UC3);


	for (int cycVec = 1; cycVec < pastRelPosVects.size(); cycVec++)
	{
		posX = (int)(double(windowSizeX) / 2 - (pastRelPosVects[cycVec - 1].y * scale));
		posY = windowSizeY - (int)(pastRelPosVects[cycVec - 1].x * scale);

		posXbefore = (int)(double(windowSizeX) / 2 - (pastRelPosVects[cycVec].y * scale));
		posYbefore = windowSizeY - (int)(pastRelPosVects[cycVec].x * scale);
		line(img, Point(posX, posY), Point(posXbefore, posYbefore), Scalar(255, 0, 0), 2);
	}	

	for (int cycVec = 1; cycVec < filteredPastRelPosVects.size(); cycVec++)
	{
		posX = (int)(double(windowSizeX) / 2 - (filteredPastRelPosVects[cycVec - 1].y * scale));
		posY = windowSizeY - (int)(filteredPastRelPosVects[cycVec - 1].x * scale);

		posXbefore = (int)(double(windowSizeX) / 2 - (filteredPastRelPosVects[cycVec].y * scale));
		posYbefore = windowSizeY - (int)(filteredPastRelPosVects[cycVec].x * scale);
		line(img, Point(posX, posY), Point(posXbefore, posYbefore), Scalar(0, 0, 255), 2);
	}

	
	/*
	cout << " posX: " << posX;
	cout << " posY: " << posY;
	cout << " posXbefore: " << posXbefore;
	cout << " posYbefore: " << posYbefore;
	*/
	//cout << " vx: " << relVel.x;
	//cout << " vy: " << relVel.y << endl;

	imshow("graph", img);
}