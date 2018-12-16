#pragma once
#include "opencv2/core.hpp"
#include "opencv2/video/tracking.hpp"

struct Coordinates
{
	double x;
	double y;
	double Sum;
};

class PositionEstimator
{
public:

	/* PUBLIC METHODS */
	PositionEstimator(int cameraFramePerSec);
	~PositionEstimator();

	void addNewPoseVectors(cv::Mat translationVector, cv::Mat rotationVector);	
	void calculatePosition();

	// getters
	std::vector<Coordinates> getPositionVectors() { return pastRelPosVects; };
	std::vector<Coordinates> getFilteredPosVectors() { return filteredPastRelPosVects; };

private:

	/* PRIVATE VARIABLES */

	double frameTime; // ellapsed time between two frame in the camera
	cv::Mat trVec; // translation vector calculated by the video decoder
	cv::Mat rotVec; // rotation vector calculated by the video decoder
	std::vector<cv::Mat> pastRotVects;
	
	// relative position and rotation to the marker
	Coordinates relVel; 
	Coordinates relPos;

	// last relative pose vectors to the marker
	std::vector<Coordinates> pastTranslateVects;
	std::vector<Coordinates> pastRelPosVects;
	
	
	Coordinates translateVectOrig; // original translate vector --> this will give the origin
	Coordinates camPoseOrig;

	// for Kalman-filter
	cv::KalmanFilter KF;
	cv::Mat_<float> measurement;
	std::vector<cv::Point> kalmanv;
	std::vector<Coordinates> filteredPastRelPosVects;
	Coordinates filteredRelPos;

	/* PRIVATE METHODS */

	void initKalmanFilter();
	
	void calcRelPosRelVel();

	void updateWithKalmanFilter();


};

