#pragma once

#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"


class VideoDecoder
{
public:
	/* PUBLIC METHODS */
	VideoDecoder(cv::Size chessboardIntersections, float chessboardSqueareDimension);
	~VideoDecoder();
	
	bool decodeFrame();

	cv::Mat getRotVec() { return rVec; };

	cv::Mat getTrVec() { return tVec; };

	cv::Mat getFrame() { return capturedFrame; };

	bool nextFrame();

private: 

	/* PRIVATE VARIABLES */
	cv::VideoCapture* capture; // webcam's capturer
	cv::Mat capturedFrame; // valid captured frame at the moment
	cv::Mat grayFrame; // grayscale version of the valid captured frame

	cv::Mat camMatrix; // const - a camera-specific matrix, which is given by the camera calibration.
	cv::Mat distCoefMatrix; // const - a camera-specific matrix, which is given by the camera calibration.
	cv::Mat rVec; // calculated rotation vector of the marker object
	cv::Mat tVec; // calculated translation vector of the marker object
	
	std::vector<cv::Point3f> modelPoints; // calculated real positions of the chessboard (in a plane)
	
	cv::Size chessboardDimensions; // dimensions of the chessboard (numbers of the intersections)
	float calibrationSquareDimension; // measured width of each square in the chessboard
	
	/* PRIVATE METHODS */

	void buildCamCalibMatrices();

	void createKnownChessboardPoints();
};

