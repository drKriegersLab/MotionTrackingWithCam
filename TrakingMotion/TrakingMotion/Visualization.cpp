#include "pch.h"
#include "Visualization.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

Visualization::Visualization(string title)
{
	setTitle(title);
}


Visualization::~Visualization()
{
}

void Visualization::setTitle(string title) {
	windowTitle = title;
}


void Visualization::addFrame(Mat newCamFrame) {
	displayedCameraFrame = newCamFrame;
}

void Visualization::dispFrame() {
	imshow(windowTitle, displayedCameraFrame);
}
