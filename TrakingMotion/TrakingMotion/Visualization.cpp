#include "pch.h"
#include "Visualization.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "PositionEstimator.h"
#include <iostream>

using namespace std;
using namespace cv;


Visualization::Visualization()
{	
	initCamWindow();
	initGraph();	

}


Visualization::~Visualization()
{
}

/* Initialization of the camera's window */
void Visualization::initCamWindow() {
	
	camWindowTitle = "CamWindow";
	namedWindow(camWindowTitle);	
}

/*
* Initialization of the graph. 
* - set the default values
* - set background
* - set a grid, where each section means 1 meter
* - put the first frame to the window
*/
void Visualization::initGraph() {

	graphWindowSizeX = 500;
	graphWindowSizeY = 500;
	scale = 100; // 1 [m] is 100 px;
	realPosColor = Scalar(255, 0, 0);
	filteredPosColor = Scalar(0, 0, 255);
	lineWidth = 2;

	namedWindow("graph");
	graphFrame = Mat::zeros(graphWindowSizeX, graphWindowSizeY, CV_8UC3);

	// set background
	Vec3b color = Vec3b(255, 255, 255);
	for (int x = 0; x < graphFrame.cols; x++) {
		for (int y = 0; y < graphFrame.rows; y++) {
			graphFrame.at<Vec3b>(Point(x, y)) = color;
		}
	}

	// set grid
	Scalar gridColor = Scalar(125, 125, 125);
	for (int y = 0; y < graphWindowSizeY; y += scale) {
		line(graphFrame, Point(0, y), Point(graphWindowSizeX, y), gridColor, 2);
	}

	for (int x = (int)(graphWindowSizeX / 2); x > 0; x -= scale) {
		line(graphFrame, Point(x, 0), Point(x, graphWindowSizeY), gridColor, 2);
	}
	for (int x = (int)(graphWindowSizeX / 2); x < graphWindowSizeX; x += scale) {
		line(graphFrame, Point(x, 0), Point(x, graphWindowSizeY), gridColor, 2);
	}

	imshow("graph", graphFrame);
}


/* Method for adding the last captured camera frame */
void Visualization::addFrameToGraphWindow(Mat newCamFrame) {
	displayedCameraFrame = newCamFrame;
}

/* Method for showing the last captured camera frame */
void Visualization::dispFrame() {
	imshow(camWindowTitle, displayedCameraFrame);
}

/*
* Method for drawing the changes to the second display. From the stored measured and filtered position vectors, get the last two elements, and 
* create a line from them, and draw those to the frame (only the last change is enough, if we don't want to erase the frame)
*/
void Visualization::showGraph(std::vector<Coordinates> relativePositions, std::vector<Coordinates> filteredPositions) {
	
	int posX, posY, posXbefore, posYbefore;
	
	if (relativePositions.size() > 2) {

		// create and draw the last chagne of the measured line
		posX = (int)(double(graphWindowSizeX) / 2 - (relativePositions[relativePositions.size() - 2].y * scale));
		posY = graphWindowSizeY - (int)(relativePositions[relativePositions.size() - 2].x * scale);

		posXbefore = (int)(double(graphWindowSizeX) / 2 - (relativePositions[relativePositions.size() - 1].y * scale));
		posYbefore = graphWindowSizeY - (int)(relativePositions[relativePositions.size() - 1].x * scale);

		line(graphFrame, Point(posX, posY), Point(posXbefore, posYbefore), realPosColor, lineWidth);

		// create and draw the last change of the filtered line
		posX = (int)(double(graphWindowSizeX) / 2 - (filteredPositions[relativePositions.size() - 2].y * scale));
		posY = graphWindowSizeY - (int)(filteredPositions[relativePositions.size() - 2].x * scale);

		posXbefore = (int)(double(graphWindowSizeX) / 2 - (filteredPositions[relativePositions.size() - 1].y * scale));
		posYbefore = graphWindowSizeY - (int)(filteredPositions[relativePositions.size() - 1].x * scale);


		line(graphFrame, Point(posX, posY), Point(posXbefore, posYbefore), filteredPosColor, lineWidth);
	}

	imshow("graph", graphFrame);
	
}