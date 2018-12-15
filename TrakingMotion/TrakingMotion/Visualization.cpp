#include "pch.h"
#include "Visualization.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "PositionEstimator.h"
#include <iostream>

using namespace std;
using namespace cv;


Visualization::Visualization(string outputPath, VideoCapture* inputVideo)
{	
	graphWindowSizeHeight = 640;
	graphWindowSizeWidth = 640;

	writeActive = true;
	//videoWriter = new VideoWriter(outputPath, inputVideoForucc, 50, Size(640, 480));
	initCamWindow();
	
	if (writeActive) {	
		int codec = inputVideo->get(CV_CAP_PROP_FOURCC);
		int fps = inputVideo->get(CV_CAP_PROP_FPS);

		videoWriter.open(outputPath, codec, fps, Size(graphWindowSizeHeight, graphWindowSizeWidth));
		
	}

	initGraph();
}


Visualization::~Visualization()
{	
	//videoWriter.release();
	//delete this;
}

void Visualization::release() {
	videoWriter.release();
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

	
	scale = 200; // 1 [m] is x px;
	realPosColor = Scalar(255, 0, 0);
	filteredPosColor = Scalar(0, 0, 255);
	lineWidth = 2;

	namedWindow("graph");
	graphFrame = Mat::zeros(graphWindowSizeWidth, graphWindowSizeHeight, CV_8UC3);

	// set background
	Vec3b color = Vec3b(255, 255, 255);
	for (int x = 0; x < graphFrame.cols; x++) {
		for (int y = 0; y < graphFrame.rows; y++) {
			graphFrame.at<Vec3b>(Point(x, y)) = color;
		}
	}

	// set grid
	Scalar gridColor = Scalar(125, 125, 125);	
	for (int y = graphWindowSizeHeight; y > 0; y -= scale) {
		line(graphFrame, Point(0, y), Point(graphWindowSizeWidth, y), gridColor, 2); // horizontal lines
	}

	for (int x = (int)(graphWindowSizeWidth / 2); x > 0; x -= scale) {
		line(graphFrame, Point(x, 0), Point(x, graphWindowSizeHeight), gridColor, 2); // vertical lines in the negative half
	}
	for (int x = (int)(graphWindowSizeWidth / 2); x < graphWindowSizeWidth; x += scale) {
		line(graphFrame, Point(x, 0), Point(x, graphWindowSizeHeight), gridColor, 2); // vertical lines in the positive half
	}

	imshow("graph", graphFrame);

	if (writeActive) {		
		videoWriter.write(graphFrame);
	}
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
		
		// current pos
		posX = (int)(double(graphWindowSizeWidth) / 2 - (relativePositions[relativePositions.size() - 1].y * scale));
		posY = graphWindowSizeHeight - (int)(relativePositions[relativePositions.size() - 1].x * scale);
		/*
		// last pos
		posXbefore = (int)(double(graphWindowSizeWidth) / 2 - (relativePositions[relativePositions.size() - 2].y * scale));
		posYbefore = graphWindowSizeHeight - (int)(relativePositions[relativePositions.size() - 2].x * scale);
		*/

		putText(graphFrame, "x", Point2f(posX, posY), FONT_HERSHEY_PLAIN, 1, realPosColor, 1);
		//line(graphFrame, Point(posX, posY), Point(posXbefore, posYbefore), realPosColor, lineWidth);

		// create and draw the last change of the filtered line
		posX = (int)(double(graphWindowSizeWidth) / 2 - (filteredPositions[relativePositions.size() - 1].y * scale));
		posY = graphWindowSizeHeight - (int)(filteredPositions[relativePositions.size() - 1].x * scale);

		posXbefore = (int)(double(graphWindowSizeWidth) / 2 - (filteredPositions[relativePositions.size() - 2].y * scale));
		posYbefore = graphWindowSizeHeight - (int)(filteredPositions[relativePositions.size() - 2].x * scale);

		line(graphFrame, Point(posX, posY), Point(posXbefore, posYbefore), filteredPosColor, lineWidth);
	}

	imshow("graph", graphFrame);
	if (writeActive) {
		videoWriter.write(graphFrame);
	}
	
}