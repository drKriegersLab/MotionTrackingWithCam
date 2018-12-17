#include "pch.h"
#include "PositionEstimator.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>

#define USING_RODRIGUES_FORMULA 0

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
	setIdentity(KF.processNoiseCov, Scalar::all(1e-2));
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
	translateVectCurr.x = trVec.at<double>(2, 0);
	translateVectCurr.y = -trVec.at<double>(0, 0);

	#if USING_RODRIGUES_FORMULA
	Mat rot;

	if (pastRelPosVects.size() > 1) {
		bool invalid = false;

		double value1, value2;

		for (int cycCol = 0; cycCol < pastRotVects[pastRotVects.size() - 1].cols; cycCol++) {
			value1 = rotVec.at<double>(cycCol, 0);
			value2 = pastRotVects[pastRotVects.size() - 1].at<double>(cycCol, 0);

			if ((value1 - (-value2)) < 0.25) // (value - (-value)
				invalid = true;
		}
		if (invalid) {
			//cout << "corrected" << endl;
			rotVec = pastRotVects[pastRotVects.size() - 1];
		}
	}
	pastRotVects.push_back(rotVec);
	
	waitKey(5);
	Rodrigues(rotVec, rot);

	Mat camTr = -rot.t()*trVec;
	Coordinates camPoseVectCurr;
	camPoseVectCurr.x = camTr.at<double>(2, 0);
	camPoseVectCurr.y = -camTr.at<double>(0, 0);

	#endif // USING_RODRIGUEZ_FORMULA

	// if there are enough captured data --> calculate the relative position and velocity
	if (pastRelPosVects.size() > 2) {
		
		// for rodriguez
		#if USING_RODRIGUES_FORMULA
		relPos.x = - translateVectOrig.x + translateVectCurr.x;
		relPos.y = -translateVectOrig.y + translateVectCurr.y;		//relPos.y = - translateVectOrig.y - translateVectCurr.y;
		#endif	

		// delta between the original and the new position: original translate vector + (-current translate vector) <in the "vehicle" coordinate system>
		relPos.x = translateVectOrig.x - translateVectCurr.x;
		relPos.y = translateVectOrig.y - translateVectCurr.y;

		relPosLast = pastRelPosVects.back();
		
		relVel.x = (relPos.x - relPosLast.x) / frameTime;
		relVel.y = (relPos.y - relPosLast.y) / frameTime;
		relVel.Sum = sqrt(relVel.x * relVel.x + relVel.y * relVel.y);
	}
	else { 
		// if we not caputred enough data --> store the current one as origin + set the relative position to zero 
		// this will be the origin of the coordinate system				
		translateVectOrig.x = translateVectCurr.x;
		translateVectOrig.y = translateVectCurr.y;
		
		#if USING_RODRIGUES_FORMULA
		camPoseOrig.x = camPoseVectCurr.x;
		camPoseOrig.y = camPoseVectCurr.y;
		#endif

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

}

