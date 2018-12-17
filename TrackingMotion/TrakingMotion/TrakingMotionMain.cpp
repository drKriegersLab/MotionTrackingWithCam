
#include "pch.h"
#include <iostream>
#include <sstream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/calib3d.hpp"

#include "VideoDecoder.h"
#include "Visualization.h"
#include "PositionEstimator.h"

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {
	
	bool readFromFile;
	string videoPath;
	string outputVideoPath;
	string calibPath;
	int interSectHor, interSectVer;
	float calibrationSquareDimension; // = 0.046f;
	const String keys =
		"{help || show the possible arguments}"
		"{inputType | file | input type: file/webcam} "		
		"{source || UNC path of the loaded file}"
		"{output || UNC path of the ouput video file}"
		"{calibPath | canon_cam_calib_pars_hd_3x3.txt | path of the calibration file }"
		"{intersectHor | 3 | horizontal intersection number of the chessboard marker}"
		"{intersectVer | 3 | horizontal intersection number of the chessboard marker}"
		"{squaredim | 0.046 | size of each square in the chessboard marker }";


	CommandLineParser parser(argc, argv, keys);
	
	// HELP
	if (parser.has("help")) {
		parser.printMessage();	
		return 1;
	}
	
	//  INPUT 
	if (parser.has("inputType")) {
		if (parser.get<string>("inputType") == "file") {
			readFromFile = true;
			cout << "[INPUT] input type: file"<< endl;
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
	
	// OUTPUT
	if (parser.has("output")) {
		outputVideoPath = parser.get<string>("output");
		cout << "[INPUT] specified output video path: " << outputVideoPath << endl;
	}
	else {
		cout << "[WRONG INPUT] output path had not been specified" << endl;
		parser.printMessage();
		return 1;
	}

	// CALIBRATION PATH
	if (parser.has("calibPath")) {
		calibPath = parser.get<string>("calibPath");
		cout << "[INPUT] calibration file: " << calibPath << endl;
	}
	else {
		cout << "[WRONG INPUT] calibration file not specified " << endl;
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
	
	//string videoPath = "c:\\__DATA__\\00_PetProjects\\MovingDetection\\MVI_7646.MP4";
	//string outputVideoPath = "c:\\__DATA__\\00_PetProjects\\MovingDetection\\output.MP4";
	
	Size chessBoardIntersections = Size(interSectVer, interSectHor);
	
	// init modules	
	VideoDecoder decoder = VideoDecoder(readFromFile, calibPath, chessBoardIntersections, calibrationSquareDimension, videoPath);
	string logFileName = "measLog.csv";
	Visualization visualization = Visualization(outputVideoPath, decoder.getVidCaptureObject(), true);
	PositionEstimator estimator = PositionEstimator(20);
	
	
	bool isFrameValid;
	if (readFromFile) {
		decoder.readAllFrames();
		vector<Mat> frames = decoder.getcapturedFrames();
		cout << "decoding frames ... " << endl;
		for (int cycFrame = 0; cycFrame < frames.size(); cycFrame++) {
			isFrameValid = decoder.decodeGivenFrame(frames[cycFrame]);
			if (isFrameValid) {
				estimator.addNewPoseVectors(decoder.getTrVec(), decoder.getRotVec());
				estimator.calculatePosition();

				visualization.updateGraph(estimator.getPositionVectors(), estimator.getFilteredPosVectors());
			}

			visualization.addFrameToGraphWindow(decoder.getFrame());
			visualization.dispFrame(cycFrame, isFrameValid);

			int keyboard = waitKey(1);

			// escape
			if (keyboard == 'q' || keyboard == 27)
				break;
		}
		cout << "no more frames" << endl;
		visualization.saveDataToMeasLog("measlog.txt");
	}
	else {
		int cycFrame = 0;
		while (true) {

			// get next frame
			bool isCaptureSuccessfull = decoder.nextFrame();
			if (isCaptureSuccessfull) {
				cycFrame++;

				// decoding frame 
				isFrameValid = decoder.decodeFrame();

				if (isFrameValid) {
					estimator.addNewPoseVectors(decoder.getTrVec(), decoder.getRotVec());
					estimator.calculatePosition();

					visualization.updateGraph(estimator.getPositionVectors(), estimator.getFilteredPosVectors());

				}
				else {
					cout << "frame is not valid" << endl;
				}

				// visalization 
				visualization.addFrameToGraphWindow(decoder.getFrame());
				visualization.dispFrame(cycFrame, isFrameValid);

				int keyboard = waitKey(1);

				// escape				
				if (keyboard == 'q' || keyboard == 27) {
					cout << "keyboard interrupt" << endl;
					break;
				}
			}
			else {
				cout << "no more frames" << endl;
				break;
			}
		}
	}

	decoder.release();
	visualization.release();
	
	return 0;

}
