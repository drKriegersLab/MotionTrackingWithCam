
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
//#include "opencv2/video/background_segm.hpp"

using namespace std;
using namespace cv;

int main()
{
	Mat frame;
	Mat fgMask;

	Ptr<BackgroundSubtractor> pBackSub;
	pBackSub = createBackgroundSubtractorMOG2();
	
	float camMatrix_ [3] [3] = { 
					{ 1601.3, 0, -9.00041}, 
					{0, 2724.92, 8.27411},
					{0, 0, 1}
						};
	float distCoefMatrix_[8] = { -0.321751f, 4.55161f, -0.0737929f, -0.170011f, -9.47005f, 0, 0, 0 };

	Mat camMatrix; //= Mat(3, 3, CV_64F, data);
	camMatrix = Mat(3, 3, CV_64F);

	camMatrix.at<double>(0, 0) = 1601.3;
	camMatrix.at<double>(0, 1) = 0;
	camMatrix.at<double>(0, 2) = -9.00041;

	camMatrix.at<double>(1, 0) = 0;
	camMatrix.at<double>(1, 1) = 2724.92;
	camMatrix.at<double>(1, 2) = 8.27411;

	camMatrix.at<double>(2, 0) = 0;
	camMatrix.at<double>(2, 1) = 0;
	camMatrix.at<double>(2, 2) = 1;



	
	Mat distCoefMatrix;
	distCoefMatrix = Mat(8, 1, CV_64F);

	distCoefMatrix.at<double>(0, 0) = -0.321751;
	distCoefMatrix.at<double>(1, 0) = 4.55161;
	distCoefMatrix.at<double>(2, 0) = -0.0737929;
	distCoefMatrix.at<double>(3, 0) = -0.170011f;
	
	distCoefMatrix.at<double>(4, 0) = -9.47005;
	distCoefMatrix.at<double>(5, 0) = 0;
	distCoefMatrix.at<double>(6, 0) = 0;
	distCoefMatrix.at<double>(7, 0) = 0;	
	
	VideoCapture capture(0);

	vector<Point3f> modelPoints; // = worldSpaceCornerPoints[0];
	float calibrationSquareDimension = 0.02f;

	Size chessboardDimensions = Size(7, 7);
	for (int i = 0; i < chessboardDimensions.height; i++)
	{		
		for (int j = 0; j < chessboardDimensions.width; j++)
		{
			
			modelPoints.push_back(Point3f(j*calibrationSquareDimension, i*calibrationSquareDimension, 0.0f));
			
			//cout << modelPoints.back();
			//cout << "num of corners: " << modelPoints.size() << endl;
		}

		cout << endl;
	}

	while (true) {
		capture >> frame;
		if (frame.empty())
			break;
		
		bool found = false;
		
		std::vector<cv::Point2f> foundPoints;

		Mat grayFrame;
		cvtColor(frame, grayFrame, CV_BGR2GRAY);

		found = findChessboardCorners(grayFrame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
		Mat rVec = Mat::zeros(3, 1, CV_64FC1);
		Mat tVec = Mat::zeros(3, 1, CV_64FC1);

		Mat omegaVec = Mat::zeros(3, 1, CV_64FC1);
		Mat vVec = Mat::zeros(3, 1, CV_64FC1);
		
		Mat rVecLast = Mat::zeros(3, 1, CV_64FC1);
		Mat tVecLast = Mat::zeros(3, 1, CV_64FC1);

		if (found)
		{
			TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001);
			Size zerozone = Size(-1, 1);
			Size winSize = Size(7, 7);

			//goodFeaturesToTrack(grayFrame, )
			cornerSubPix(grayFrame, foundPoints, winSize, zerozone, criteria);
			//Mat objP = Mat(6 * 7, 3, CV_64F);
			//vector<cv::Point3f> modelPoints;
			rVecLast = rVec;
			tVecLast = tVec;

			rVec = Mat::zeros(3, 1, CV_64FC1);
			tVec = Mat::zeros(3, 1, CV_64FC1);

			solvePnPRansac(modelPoints, foundPoints, camMatrix, distCoefMatrix, rVec, tVec);

			for (int rownum = 0; rownum < rVec.rows; rownum++)
			{
				double a = tVec.at<double>(rownum, 0);
				double b = tVecLast.at<double>(rownum, 0);
				vVec.at<double>(rownum, 0) = (double)(a - b);
			}
						
			//cout << "rVec[0]: " << rVec.at<double>(0, 0) << endl;
			
			//cout << "rVec[1]: " << rVec.at<double>(1, 0) << endl;

			double angleDegree = (rVec.at<double>(0, 0) * 180) / CV_PI;
			cout << "angle: " << angleDegree << endl;

			//cout << "rVec[2]: " << rVec.at<double>(2, 0) << endl;
			
			//cout << "tVec[0]: " << tVec.at<double>(0, 0) << endl;
			//cout << "tVec[1]: " << tVec.at<double>(1, 0) << endl;
			//cout << "tVec[0]: " << tVec.at<double>(1, 0) << endl;

			
			vector<Point2f> projectedPoints;
			vector<Point2f> imgPointVect;
			Mat imgPoints = Mat::zeros(3, 2, CV_64FC1);


			//projectPoints(modelPoints, rVec, tVec, camMatrix, distCoefMatrix, projectedPoints, imgPoints);
			/*
			cout << "num of points: " << projectedPoints.size() << endl;
			cout << "point 0.x: " << projectedPoints[0].x << endl;
			cout << "point 0.y: " << projectedPoints[0].x << endl;

			cout << "point 1.x: " << projectedPoints[1].x << endl;
			cout << "point 1.y: " << projectedPoints[1].y << endl;

			cout << "point 2.x: " << projectedPoints[2].x << endl;
			cout << "point 2.y: " << projectedPoints[2].y << endl;

			cout << "num of imgPoints: " << imgPoints.size() << endl;
			cout << "imgPoints 0.x: " << imgPoints.at<double>(0,0)   << endl;
			cout << "imgPoints 0.y: " << imgPoints.at<double>(0, 1) << endl;
			imgPointVect.push_back(Point2f(imgPoints.at<double>(0, 0), imgPoints.at<double>(0, 1)));

			cout << "imgPoints 1.x: " << imgPoints.at<double>(1, 0) << endl;
			cout << "imgPoints 1.y: " << imgPoints.at<double>(1, 1) << endl;
			imgPointVect.push_back(Point2f(imgPoints.at<double>(1, 0), imgPoints.at<double>(1, 1)));
			
			cout << "imgPoints 2.x: " << imgPoints.at<double>(2, 0) << endl;
			cout << "imgPoints 2.y: " << imgPoints.at<double>(2, 1) << endl;
			imgPointVect.push_back(Point2f(imgPoints.at<double>(2, 0), imgPoints.at<double>(2, 1)));

			*/


			//Point2f pointOrig = projectedPoints[0];			
			//line(frame, pointOrig, imgPointVect[2], Scalar(255, 0, 0), 5);
			//line(frame, pointOrig, imgPointVect[2], Scalar(0, 255, 0), 5);
			//line(frame, pointOrig, imgPointVect[2], Scalar(0, 0, 255), 5);
			//Point2f pointOrig = projectedPoints[24];
			//line(frame, pointOrig, imgPointVect[2], Scalar(255, 0, 0), 5);
			//line(frame, Point2f(0,0), imgPointVect[2], Scalar(0, 255, 0), 5);
			//line(frame, Point2f(0,0), imgPointVect[2], Scalar(0, 0, 255), 5);


			//imshow("img", frame);
		}
		//cornerSubPix()

		pBackSub->apply(frame, fgMask);		

		imshow("Frame", frame);
		//imshow("Frame", dst_norm_scaled);
		//imshow("FG Mask", fgMask);

		int keyboard = waitKey(30);
		if (keyboard == 'q' || keyboard == 27)
			break;


		/*
		vector<vector<Point>> contours;
		vector<Vec4i> hierarcy;
		Mat cannyOut;

		int thresh = 100000;
		Canny(fgMask, cannyOut, thresh, thresh * 2, 3);

		findContours(fgMask, contours, hierarcy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		Mat drawing = Mat::zeros(cannyOut.size(), CV_8UC3);

		RNG rng(12345);

		vector<Rect> boundRect(contours.size());

		int i_max = 5;
		//if (i_max > contours.size())
			i_max = contours.size();

		for (int i = 0; i < i_max; i++) {		
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			//drawContours(drawing, contours, i, color, 2, 8, hierarcy, 0, Point());

			boundRect[i] = boundingRect(contours[i]);
			rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
		}

		namedWindow("contours", CV_WINDOW_AUTOSIZE);
		imshow("contours", drawing);
		*/
	}

	return 0;
}
