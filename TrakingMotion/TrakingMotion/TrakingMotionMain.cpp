
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
	Size chessBoardIntersections = Size(7, 7);
	float calibrationSquareDimension = 0.02f;
	Mat r, t;

	VideoDecoder decoder = VideoDecoder(chessBoardIntersections, calibrationSquareDimension);
	Visualization visualization = Visualization("display");
	PositionEstimator estimator = PositionEstimator();

	bool isPicValid = false;
	bool isFrameValid = false;

	while (true) {
		
		isFrameValid = decoder.nextFrame();

		if (isFrameValid) {

			/* decoding frame */
			isPicValid = decoder.decodeFrame();

			if (isPicValid) {
				
				r = decoder.getRotVec();
				t = decoder.getTrVec();

				//cout << "tr: " << t.at<double>(0, 0) << ", " << t.at<double>(1, 0) << ", " << t.at<double>(2, 0) << endl;
				//cout << "rt: " << r.at<double>(0, 0) << ", " << r.at<double>(1, 0) << ", " << r.at<double>(2, 0) << endl;

				estimator.addNewPoseVectors(t, r);
				estimator.calculatePosition();

			}

			else {
				cout << "frame is not valid" << endl;
			}			
			
			/* visalization */
			visualization.addFrame(decoder.getFrame());
			visualization.dispFrame();			

		}		
		else {
			cout << "no more frames" << endl;
			break;
		}	
	}
	return 0;

}
