#pragma once
#include "opencv2/core.hpp"
#include "PositionEstimator.h"
#include "opencv2/videoio.hpp"

class Visualization
{
public:
	/* PUBLIC METHODS */
	Visualization(std::string outputPath, cv::VideoCapture* inputVideo, bool writeVideo);
	~Visualization();

	void updateGraph(std::vector<Coordinates> relativePositions, std::vector<Coordinates> filteredPositions); // graph visualization
	void addFrameToGraphWindow(cv::Mat newGraphFrame); // add captured picture
	void dispFrame(int numOfFrames, bool captured); // show captured picture
	void release();

private:
	/* PRIVATE VARIABLES */
	bool writeActive;

	cv::Mat displayedCameraFrame; // the captured camera frame which will be shown	

	/* for graph displaying */
	// size the graph window in pixels
	int graphFrameSizeHeight; 
	int graphFrameSizeWidth;
	int summaryFrameHeight;
	int summaryFrameWidth;
	int newCamFrameHeight;
	int newCamFrameWidth;	
	int scale; // scaling factor --> multiply the coordinates with it to a better picture

	cv::Mat graphFrame; // canvas of the graph
	
	Coordinates realPos, filteredPos;
	
	// basic drawing settings
	cv::Scalar realPosColor;
	cv::Scalar filteredPosColor;
	int lineWidth;

	cv::VideoWriter videoWriter;

	/* PRIVATE METHODS */
	void initGraphFrame();
	
	
};

