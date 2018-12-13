#pragma once
#include "opencv2/core.hpp"
#include "PositionEstimator.h"

class Visualization
{
public:
	/* PUBLIC METHODS */
	Visualization();
	~Visualization();

	void showGraph(std::vector<Coordinates> relativePositions, std::vector<Coordinates> filteredPositions); // graph visualization
	void addFrameToGraphWindow(cv::Mat newGraphFrame); // add captured picture
	void dispFrame(); // show captured picture

private:
	/* PRIVATE VARIABLES */
	cv::Mat displayedCameraFrame; // the captured camera frame which will be shown
	
	std::string camWindowTitle;		

	/* for graph displaying */
	// size the graph window in pixels
	int graphWindowSizeX; 
	int graphWindowSizeY;
	int scale; // scaling factor --> multiply the coordinates with it to a better picture

	cv::Mat graphFrame; // canvas of the graph
	
	// basic drawing settings
	cv::Scalar realPosColor;
	cv::Scalar filteredPosColor;
	int lineWidth;

	/* PRIVATE METHODS */
	
	void initCamWindow();
	void initGraph();
	
};

