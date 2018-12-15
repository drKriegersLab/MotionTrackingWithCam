
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

int main()
{

	//string calibPath = "c:\\__DATA__\\00_PetProjects\\MovingDetection\\MotionTrackingWithCam\\calib\\calib\\canon_cam_calib_pars.txt";
	//string calibPath = "c:\\__DATA__\\00_PetProjects\\MovingDetection\\MotionTrackingWithCam\\calib\\calib\\canon_cam_calib_pars_vga.txt";
	string calibPath = "c:\\__DATA__\\00_PetProjects\\MovingDetection\\MotionTrackingWithCam\\calib\\calib\\canon_cam_calib_pars_vga_3x3.txt";
	//Size chessBoardIntersections = Size(7, 7);
	Size chessBoardIntersections = Size(3, 3);
	//float calibrationSquareDimension = 0.02f;
	float calibrationSquareDimension = 0.046f;
	string videoPath = "c:\\__DATA__\\00_PetProjects\\MovingDetection\\MVI_7637.MP4";
	string outputVideoPath = "c:\\__DATA__\\00_PetProjects\\MovingDetection\\output.MP4";

	Mat r, t;
	
	VideoDecoder decoder = VideoDecoder(calibPath, chessBoardIntersections, calibrationSquareDimension, videoPath);
	Visualization visualization = Visualization(outputVideoPath, decoder.getVidCaptureObject(), true);
	PositionEstimator estimator = PositionEstimator(20);
	
	bool isPicValid = false;
	bool isFrameValid = false;
	
	decoder.readAllFrames();
	vector<Mat> frames = decoder.getcapturedFrames();

	for (int cycFrame = 0; cycFrame < frames.size(); cycFrame++)
	{
		bool valid = decoder.decodeGivenFrame(frames[cycFrame]);
		if (valid)
		{
			r = decoder.getRotVec();
			t = decoder.getTrVec();

			estimator.addNewPoseVectors(t, r);
			estimator.calculatePosition();

			visualization.updateGraph(estimator.getPositionVectors(), estimator.getFilteredPosVectors());
		}

		visualization.addFrameToGraphWindow(decoder.getFrame());
		visualization.dispFrame(cycFrame, valid);

		int keyboard = waitKey(1);
		if (keyboard == 'q' || keyboard == 27)
			break;

	}
	
	
	cout << "no more frames" << endl;
	/*

	while (true) {
		
		isFrameValid = decoder.nextFrame();

		if (isFrameValid) {

			// decoding frame 
			isPicValid = decoder.decodeFrame();

			if (isPicValid) {
				
				r = decoder.getRotVec();
				t = decoder.getTrVec();

				//cout << "tr: " << t.at<double>(0, 0) << ", " << t.at<double>(1, 0) << ", " << t.at<double>(2, 0) << endl;
				//cout << "rt: " << r.at<double>(0, 0) << ", " << r.at<double>(1, 0) << ", " << r.at<double>(2, 0) << endl;

				estimator.addNewPoseVectors(t, r);
				estimator.calculatePosition();

				visualization.showGraph(estimator.getPositionVectors(), estimator.getFilteredPosVectors());


			}

			else {
				cout << "frame is not valid" << endl;
			}			
			
			// visalization 
			visualization.addFrameToGraphWindow(decoder.getFrame());
			visualization.dispFrame();			

			int keyboard = waitKey(1);
			if (keyboard == 'q' || keyboard == 27)
				break;



		}		
		else {
			cout << "no more frames" << endl;
			break;
		}	
	}
	*/
	decoder.release();
	visualization.release();
	return 0;

}
