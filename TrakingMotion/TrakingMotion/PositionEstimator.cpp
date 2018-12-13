#include "pch.h"
#include "PositionEstimator.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

PositionEstimator::PositionEstimator(int cameraFramePerSec) {
	
	frameTime = 1 / (double)cameraFramePerSec;

	initKalmanFilter();		
}

PositionEstimator::~PositionEstimator()
{
}

/* Method for setting the Kalman-filter up */
void PositionEstimator::initKalmanFilter() {
	KF = KalmanFilter(4, 2, 0);
	KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
	
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

/* Method for updating the pose vectors */
void PositionEstimator::addNewPoseVectors(Mat translationVector, Mat rotationVector) {
	trVec = translationVector;
	rotVec = rotationVector;
}

/* Method for managing the position calculation */
void PositionEstimator::calculatePosition() {

	calcRelPosRelVel();
	updateWithKalmanFilter();

}


/* Method for calculating the relative position to the marker */
void PositionEstimator::calcRelPosRelVel() {
	Coordinates relPosLast;
	Coordinates translateVectCurr;


	// convert from the camera coordinate system to the vehicle coordinate system and store the new values
	translateVectCurr.x = -trVec.at<double>(2, 0);
	translateVectCurr.y = trVec.at<double>(0, 0);
	pastTranslateVects.push_back(translateVectCurr);

	// if there are enough captured data --> calculate the relative position and velocity
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
	}
	else { 
		// if we not caputred enough data --> store the current one as origin + set the relative position to zero 
		// this will be the origin of the coordinate system
		translateVectOrig.x = translateVectCurr.x;
		translateVectOrig.y = translateVectCurr.y;

		relPos.x = 0;
		relPos.y = 0;
		relPos.Sum = 0;
	}

	pastRelPosVects.push_back(relPos);
}


void PositionEstimator::updateWithKalmanFilter() {
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
}

