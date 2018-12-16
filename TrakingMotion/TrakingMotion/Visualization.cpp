#include "pch.h"
#include "Visualization.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "PositionEstimator.h"
#include <iostream>

using namespace std;
using namespace cv;

/*
* Init Visualization class:
*	- set default frame sizes
*	- init video writer class
*	- init graph
*/
Visualization::Visualization(string outputPath, VideoCapture* inputVideo, bool writeVideo) {	
	// specifying each frame's dimensions
	graphFrameSizeHeight = 640;
	graphFrameSizeWidth = 640;
	
	newCamFrameHeight = graphFrameSizeHeight;
	newCamFrameWidth = (int)(inputVideo->get(CAP_PROP_FRAME_WIDTH) / (inputVideo->get(CAP_PROP_FRAME_HEIGHT) / graphFrameSizeHeight));
	
	summaryFrameWidth = graphFrameSizeWidth + newCamFrameWidth;
	summaryFrameHeight = graphFrameSizeHeight;
	
	writeActive = writeVideo;
	
	if (writeActive) {	
		int codec = inputVideo->get(CV_CAP_PROP_FOURCC);
		int fps = inputVideo->get(CV_CAP_PROP_FPS);

		videoWriter.open(outputPath, codec, fps, Size(summaryFrameWidth, summaryFrameHeight));		
	}

	initGraphFrame();	
}


Visualization::~Visualization() {	
	//videoWriter.release();
	//delete this;
}

void Visualization::release() {
	videoWriter.release();
}



/*
* Initialization of the graph's frame. 
* - set the default values
* - set background
* - set a grid, where each section means 1 meter
* - draw axes
*/
void Visualization::initGraphFrame() {
	
	scale = 100; // 1 [m] is x px;
	realPosColor = Scalar(255, 0, 0);
	filteredPosColor = Scalar(0, 0, 255);
	lineWidth = 2;	
	
	graphFrame = Mat::zeros(graphFrameSizeWidth, graphFrameSizeHeight, CV_8UC3);

	// set background
	Vec3b color = Vec3b(255, 255, 255);
	for (int x = 0; x < graphFrame.cols; x++) {
		for (int y = 0; y < graphFrame.rows; y++) {
			graphFrame.at<Vec3b>(Point(x, y)) = color;
		}
	}

	// set grid
	Scalar gridColor = Scalar(125, 125, 125);	
	for (int y = graphFrameSizeHeight; y > 0; y -= scale) {
		line(graphFrame, Point(0, y), Point(graphFrameSizeWidth, y), gridColor, lineWidth); // horizontal lines
	}

	for (int x = (int)(graphFrameSizeWidth / 2); x > 0; x -= scale) {
		line(graphFrame, Point(x, 0), Point(x, graphFrameSizeHeight), gridColor, lineWidth); // vertical lines in the negative half
	}
	for (int x = (int)(graphFrameSizeWidth / 2); x < graphFrameSizeWidth; x += scale) {
		line(graphFrame, Point(x, 0), Point(x, graphFrameSizeHeight), gridColor, lineWidth); // vertical lines in the positive half
	}

	// set axes
	arrowedLine(graphFrame, Point(graphFrameSizeWidth / 2, graphFrameSizeHeight), Point(graphFrameSizeWidth / 2, 0), Scalar(0, 0, 0), lineWidth, 4, 0, 0.03); // x
	arrowedLine(graphFrame, Point(graphFrameSizeWidth / 2, graphFrameSizeHeight), Point(0, graphFrameSizeHeight), Scalar(0, 0, 0), lineWidth, 8, 0, 0.03); // y
	putText(graphFrame, "axis x", Point2f(graphFrameSizeWidth / 2 - 75, 15), FONT_ITALIC, 0.5, Scalar(0, 0, 0), 1); // x
	putText(graphFrame, "axis y", Point2f(15, graphFrameSizeHeight - 15), FONT_ITALIC, 0.5, Scalar(0, 0, 0), 1); // y

}

/* Method for adding the last captured camera frame */
void Visualization::addFrameToGraphWindow(Mat newCamFrame) {
	displayedCameraFrame = newCamFrame;
}

/* Method for displaying the summary window 
*  - resize the camera frame to the graph and put them together to the summary frame
*  - put information labels to the left upper corner of the frame
*/
void Visualization::dispFrame(int numOfFrames, bool captured) {
	
	Mat summaryFrame = Mat::zeros(Size(summaryFrameWidth, summaryFrameHeight), CV_8UC3);
	
	// resize the camera frame to fit to the summary window
	Mat newDispFrame;
	resize(displayedCameraFrame, newDispFrame, Size(newCamFrameWidth, newCamFrameHeight));

	// copy the frames  to the summary frame
	newDispFrame.copyTo(summaryFrame(Rect(0, 0, newCamFrameWidth, newCamFrameHeight)));
	graphFrame.copyTo(summaryFrame(Rect(newCamFrameWidth,0, graphFrameSizeWidth, graphFrameSizeHeight)));
	

	int font = FONT_HERSHEY_COMPLEX_SMALL;
	double fontScale = 0.8;	
	int fontThickness = 1;
	int txtBoxMaxWidth, txtBoxMaxHeight;
	
	// set labels
	vector<string> labels;

	// init the first label
	string label = "frame: " + to_string(numOfFrames);	
		
	// create container box for the labels with default maximums size
	Size labelSize = getTextSize(label, font, fontScale, fontThickness, 0);
	txtBoxMaxWidth = 400;
	txtBoxMaxHeight = (labelSize.height + 5) * 5 + 5;	
	Mat txtBox = Mat::zeros(Size(txtBoxMaxWidth, txtBoxMaxHeight), CV_8UC3);
	for (int cycRow = 0; cycRow < txtBox.rows; cycRow++) {
		for (int cycCol = 0; cycCol< txtBox.cols; cycCol++) {
			txtBox.at<Vec3b>(Point(cycCol, cycRow)) = Vec3b(40, 40, 40);
		}
	}

	// add other labels
	putText(txtBox, label, Point(0, labelSize.height + 5), font, fontScale, Scalar(0, 0, 200), 1, 1);
	labels.push_back(label);

	label = "measured posX: " + to_string(realPos.x);
	putText(txtBox, label, Point(0, (labelSize.height + 5) * 2), font, fontScale, Scalar(0, 153, 204), fontThickness, 1);
	labels.push_back(label);

	label = "measured posY: " + to_string(realPos.y);
	putText(txtBox, label, Point(0, (labelSize.height + 5) * 3), font, fontScale, Scalar(0, 153, 204), fontThickness, 1);
	labels.push_back(label);

	label = "filtered posX: " + to_string(filteredPos.x);
	putText(txtBox, label, Point(0, (labelSize.height + 5) * 4), font, fontScale, Scalar(255, 153, 0), fontThickness, 1);
	labels.push_back(label);

	label = "filtered posY: " + to_string(filteredPos.y);
	putText(txtBox, label, Point(0, (labelSize.height + 5) * 5), font, fontScale, Scalar(255, 153, 0), fontThickness, 1);
	labels.push_back(label);

	// determine the necessary size of the container box and cut it
	int txtBoxWidth = 0;
	for (int cycLabel = 0; cycLabel < labels.size(); cycLabel++) {
		labelSize = getTextSize(labels[cycLabel], font, fontScale, fontThickness, 0);
		if (txtBoxWidth < labelSize.width) {
			if (labelSize.width < txtBoxMaxWidth)
				txtBoxWidth = labelSize.width;
			else
				txtBoxWidth = txtBoxMaxWidth;
		}	
	}
	
	// add the container box to the frame
	txtBox(Rect(0, 0, txtBoxWidth, txtBoxMaxHeight)).copyTo(summaryFrame(Rect(15, 15, txtBoxWidth, txtBoxMaxHeight)));

	// show and/or save
	imshow("summary", summaryFrame);

	if (writeActive) {
		videoWriter.write(summaryFrame);
	}
}

/*
* Method for drawing the changes to the second display. From the stored measured and filtered position vectors, get the last two elements, and 
* create a line from them, and draw those to the frame (only the last change is enough, if we don't want to erase the frame)
*/
void Visualization::updateGraph(std::vector<Coordinates> relativePositions, std::vector<Coordinates> filteredPositions) {
	
	 
	Coordinates filteredPosPrev;
	int posX, posY, posXbefore, posYbefore;
	
	if (relativePositions.size() > 2) {	
		realPos = relativePositions[relativePositions.size() - 1];
		filteredPos = filteredPositions[filteredPositions.size() - 1];
		filteredPosPrev = filteredPositions[filteredPositions.size() - 2];
		// create and draw the last chagne of the measured line
		
		// current pos
		posX = (int)(double(graphFrameSizeWidth) / 2 - (realPos.y * scale));
		posY = graphFrameSizeHeight - (int)(realPos.x * scale);
		/*
		// last pos
		posXbefore = (int)(double(graphWindowSizeWidth) / 2 - (relativePositions[relativePositions.size() - 2].y * scale));
		posYbefore = graphWindowSizeHeight - (int)(relativePositions[relativePositions.size() - 2].x * scale);
		*/

		putText(graphFrame, "x", Point2f(posX, posY), FONT_HERSHEY_PLAIN, 1, realPosColor, 1);
		//line(graphFrame, Point(posX, posY), Point(posXbefore, posYbefore), realPosColor, lineWidth);

		// create and draw the last change of the filtered line
		posX = (int)(double(graphFrameSizeWidth) / 2 - (filteredPos.y * scale));
		posY = graphFrameSizeHeight - (int)(filteredPos.x * scale);

		posXbefore = (int)(double(graphFrameSizeWidth) / 2 - (filteredPosPrev.y * scale));
		posYbefore = graphFrameSizeHeight - (int)(filteredPosPrev.x * scale);

		line(graphFrame, Point(posX, posY), Point(posXbefore, posYbefore), filteredPosColor, lineWidth);
	}
}