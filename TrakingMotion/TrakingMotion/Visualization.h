#pragma once
#include "opencv2/core.hpp"

class Visualization
{
public:
	/* PUBLIC METHODS */
	Visualization(std::string title);
	~Visualization();

	void setTitle(std::string title);
	void addFrame(cv::Mat newCamFrame);
	void dispFrame();

private:
	/* PRIVATE VARIABLES */
	cv::Mat displayedCameraFrame; // the captured camera frame which will be shown
	std::string windowTitle;
};

