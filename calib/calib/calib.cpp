#include "pch.h"
#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

#include "calibrate.hpp"

#include <sstream>
#include <fstream>


using namespace std;
using namespace cv;


const float calibrationSquareDimension = 0.02f; //[m]
const Size chessboardDimensions = Size(7, 7);

int main(int argc, char* argv[])
{
	/* variable definitions */
	bool readFromFile;
	string videoPath;
	string outputFile;
	int interSectHor, interSectVer;
	float calibrationSquareDimension;

	/* input parser */
	const String keys =
		"{help || show the argument list}"
		"{inputType | file | input type: file/webcam} "
		"{source || UNC path of the loaded file}"
		"{output | output.txt | specifies the output parameterlist's file}"
		"{intersectHor | 3 | horizontal intersection number of the chessboard marker}"
		"{intersectVer | 3 | horizontal intersection number of the chessboard marker}"
		"{squaredim | 0.046 | size of each square in the chessboard marker }";
	
	CommandLineParser parser(argc, argv, keys);

	if (parser.has("help")) {
		cout << "possible arguments:" << endl;
		parser.printMessage();
		cout << endl << "This program creates the camera matrices, which are essential to further image processing" << endl;
		cout << "What you need to do?" << endl;
		cout << "	- get a black and white chessboard as marker" << endl;
		cout << "	- measure the width of the squares  --> it will be the squaredim"<< endl;
		cout << "	- specify the number of intersections --> it will be the intersectHor and intersectVer" << endl;
		cout << "	- shoot a video from it or use your webcam (inputType)" << endl;
		cout << "	- specify the output file (it will be a txt with camera matrix and distance coefficients" << endl;
		return 1;
	}

	//  INPUT 
	if (parser.has("inputType")) {
		if (parser.get<string>("inputType") == "file") {
			readFromFile = true;
			cout << "[INPUT] input type: file" << endl;
		}
		else {
			readFromFile = false;
			cout << "[INPUT] input type: webcam" << endl;
		}
	}
	else {
		cout << "[WRONG INPUT] input not specified" << endl;
		parser.printMessage();
		return 1;
	}

	// SOURCE
	if (parser.has("source")) {
		videoPath = parser.get<string>("source");
		cout << "[INPUT] specified video path: " << videoPath << endl;
	}
	else {
		cout << "[WRONG INPUT] source path had not been specified" << endl;
		parser.printMessage();
		return 1;
	}
	
	//OUTPUT 	
	if (parser.has("output")) {
		outputFile = parser.get<string>("output");
		cout << "[INPUT] specified video path: " << outputFile << endl;
	}
	else {
		cout << "[WRONG INPUT] source path had not been specified" << endl;
		parser.printMessage();
		return 1;
	}

	// INTERSECTIONS NUMBER HORIZONTALLY
	if (parser.has("intersectHor")) {
		interSectHor = parser.get<int>("intersectHor");
		cout << "[INPUT] number of intersections horizontally in the chessboard " << interSectHor << endl;
	}
	else {
		cout << "[WRONG INPUT] horizontal intersection number not specified " << endl;
		parser.printMessage();
		return 1;
	}

	// INTERSECTIONS NUMBER VERTICALLY
	if (parser.has("intersectVer")) {
		interSectVer = parser.get<int>("intersectVer");
		cout << "[INPUT] number of intersections vertically in the chessboard: " << interSectVer << endl;
	}
	else {
		cout << "[WRONG INPUT] vertical intersection number not specified " << endl;
		parser.printMessage();
		return 1;
	}

	// SQUARE DIMENSION
	if (parser.has("squaredim")) {
		calibrationSquareDimension = parser.get<float>("squaredim");
		cout << "[INPUT] squaredim:  " << calibrationSquareDimension << endl;
	}
	else {
		cout << "[WRONG INPUT] squaredim not specified" << endl;
		parser.printMessage();
		return 1;
	}

	 /* main part */

	CameraCalibrator cameraCalibrator(readFromFile, videoPath, outputFile, Size(interSectHor, interSectVer), calibrationSquareDimension);
	cameraCalibrator.performCalibration();

	return 0;
    
}
