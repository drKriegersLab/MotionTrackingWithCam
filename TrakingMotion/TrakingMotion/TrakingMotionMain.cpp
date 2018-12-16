
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

#define CALIB_PARLIST_WEBCAM 0
#define CALIB_PARLIST_CANON_VGA_7X7 1
#define CALIB_PARLIST_CANON_VGA_3X3 2
#define CALIB_PARLIST_CANON_HD_3X3 3
#define CALIB_PARLIST_CANON_HD_7X7 4
#define CALIB_PARLIST_CANON_FHD_7x7 5



using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {
	
	string videoPath;
	string outputVideoPath;

	const String keys =
		"{help || show the possible arguments}"
		//"{flag | | true if the input source is specified video file. If it false, the program will use the first found webcam}"
		//"{sourcetype s | 0 | camera source type, which specifies the camera matrices (webcam/camera vga 3x3/camera hd 7x7}"
		"{source || UNC path of the loaded file}"
		"{output || UNC path of the ouput video file}";


	CommandLineParser parser(argc, argv, keys);
	
	if (parser.has("help")) {
		parser.printMessage();		
	}
	
	if (parser.has("source")) {
		videoPath = parser.get<string>("source");
		cout << "specified video path: " << videoPath << endl;
	}
	else {
		cout << "source path had not been specified";
		parser.printMessage();
		return 1;
	}

	if (parser.has("output")) {
		outputVideoPath = parser.get<string>("output");
		cout << "specified output video path: " << outputVideoPath << endl;
	}
	else {
		cout << "output path had not been specified";
		parser.printMessage();
		return 1;
	}
	
	 
	// init base variables
	bool readFromFile = true;
	int sourceType = CALIB_PARLIST_CANON_HD_3X3;
	int chessBoardIntersectionsVertical = 3;
	int chessBoardIntersectionsHorizontal = 3;
	float calibrationSquareDimension = 0.046f;
	//string videoPath = "c:\\__DATA__\\00_PetProjects\\MovingDetection\\MVI_7646.MP4";
	//string outputVideoPath = "c:\\__DATA__\\00_PetProjects\\MovingDetection\\output.MP4";

	string calibPath;
	switch (sourceType)
	{
	case CALIB_PARLIST_WEBCAM:
		calibPath = "canon_cam_calib_pars.txt";
		break;	
	case CALIB_PARLIST_CANON_VGA_7X7:
		calibPath = "canon_cam_calib_pars_vga.txt";
		break;
	case CALIB_PARLIST_CANON_VGA_3X3:
		calibPath = "canon_cam_calib_pars_vga_3x3.txt";
		break;
	case CALIB_PARLIST_CANON_HD_3X3:
		calibPath = "canon_cam_calib_pars_hd_3x3.txt";
		break;
	case CALIB_PARLIST_CANON_HD_7X7:
		calibPath = "canon_cam_calib_pars_hd_7x7.txt";
		break;
	case CALIB_PARLIST_CANON_FHD_7x7:
		calibPath = "canon_cam_calib_pars.txt";
		break;
	default: // load the webcam parameter list
		calibPath = "canon_cam_calib_pars.txt";
		break;
	}
	
	Size chessBoardIntersections = Size(chessBoardIntersectionsVertical, chessBoardIntersectionsHorizontal);
	
	// init modules	
	VideoDecoder decoder = VideoDecoder(readFromFile, calibPath, chessBoardIntersections, calibrationSquareDimension, videoPath);
	Visualization visualization = Visualization(outputVideoPath, decoder.getVidCaptureObject(), true);
	PositionEstimator estimator = PositionEstimator(20);
	
	
	bool isFrameValid;
	if (readFromFile) {
		decoder.readAllFrames();
		vector<Mat> frames = decoder.getcapturedFrames();

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
				if (keyboard == 'q' || keyboard == 27)
					break;
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
