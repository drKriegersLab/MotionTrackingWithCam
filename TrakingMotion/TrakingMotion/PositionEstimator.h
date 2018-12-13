#pragma once
#include "opencv2/core.hpp"
#include "opencv2/video/tracking.hpp"

class PositionEstimator
{
public:
	PositionEstimator();

	~PositionEstimator();

	void addNewPoseVectors(cv::Mat translationVector, cv::Mat rotationVector);

	void calculatePosition();

private:

	struct Coordinates
	{
		double x;
		double y;
		double Sum;
	};
	cv::Mat trVec;
	cv::Mat rotVec;
	std::vector<Coordinates> pastTranslateVects;
	std::vector<Coordinates> pastRelPosVects;
	
	Coordinates translateVectOrig;

	cv::KalmanFilter KF;
	cv::Mat_<float> measurement;
	std::vector<cv::Point> kalmanv;
	std::vector<Coordinates> filteredPastRelPosVects;


	void showGraph();

};

