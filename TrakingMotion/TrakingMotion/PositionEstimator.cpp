#include "pch.h"
#include "PositionEstimator.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

PositionEstimator::PositionEstimator()
{
}


PositionEstimator::~PositionEstimator()
{
}

void PositionEstimator::addNewPoseVectors(Mat translationVector, Mat rotationVector) {
	trVec = translationVector;
	rotVec = rotationVector;
}

void PositionEstimator::calculatePosition() {
	
	double camFps = 50;
	double frameTime = 1 / camFps;
	Coordinates relVel;
	Coordinates relPos;
	Coordinates relPosLast;
	Coordinates posCurr;
	Coordinates translateVectCurr;

	vector<Coordinates> diffVect;

	// convert to vehicle coordinate sys
	translateVectCurr.x = trVec.at<double>(2, 0);
	translateVectCurr.y = trVec.at<double>(0, 0);
	
	pastTranslateVects.push_back(translateVectCurr);	

	if (pastTranslateVects.size() > 2) {

		relPos.x = translateVectCurr.x - translateVectOrig.x;
		relPos.y = translateVectCurr.y - translateVectOrig.y;
		relPos.Sum = sqrt(relPos.x * relPos.x + relPos.y *relPos.y);

		relPosLast = pastRelPosVects.back();
		cout << frameTime << endl;
		relVel.x = (relPos.x - relPosLast.x) / frameTime;
		relVel.y = (relPos.y - relPosLast.y) / frameTime;
		relVel.Sum = sqrt(relVel.x * relVel.x + relVel.y * relVel.y);


		//cout << " r: " << relPos.Sum;
		//cout << " rx: " << relPos.x;
		//cout << " ry: " << relPos.y;
		//cout << " | v " << relVel.Sum;
		//cout << " vx: " << relVel.x;
		//cout << " vy: " << relVel.y << endl;


		

		showGraph();

	}
	else {
		translateVectOrig.x = translateVectCurr.x;
		translateVectOrig.y = translateVectCurr.y;

		relPos.x = 0;
		relPos.y = 0;
		relPos.Sum = 0;		
	}
	pastRelPosVects.push_back(relPos);


}

void PositionEstimator::showGraph() {
	int windowSizeX = 512;
	int windowSizeY = 512;
	int scale = 500;
	int posX, posY, posXbefore, posYbefore;

	namedWindow("graph");
	vector<Coordinates> posDiff;
	Mat img = Mat::zeros(windowSizeX, windowSizeY, CV_8UC3);


	for (int cycVec = 1; cycVec < pastRelPosVects.size(); cycVec++)
	{
		posX = (int)(double(windowSizeX) / 2 - (pastRelPosVects[cycVec - 1].y * scale));
		posY = windowSizeY - (int)(pastRelPosVects[cycVec - 1].x * scale);

		posXbefore = (int)(double(windowSizeX) / 2 - (pastRelPosVects[cycVec].y * scale));
		posYbefore = windowSizeY - (int)(pastRelPosVects[cycVec].x * scale);
		line(img, Point(posX, posY), Point(posXbefore, posYbefore), Scalar(255, 0, 0), 2);
	}	
	

	cout << " posX: " << posX;
	cout << " posY: " << posY;
	cout << " posXbefore: " << posXbefore;
	cout << " posYbefore: " << posYbefore;
	//cout << " vx: " << relVel.x;
	//cout << " vy: " << relVel.y << endl;

	imshow("graph", img);
}