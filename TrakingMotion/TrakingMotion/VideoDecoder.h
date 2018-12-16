#pragma once

#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"


class VideoDecoder
{
public:
	/* PUBLIC METHODS */
	VideoDecoder(bool readFromFile, std::string pathToCalibResult, cv::Size chessboardIntersections, float chessboardSqueareDimension, std::string pathToVideoFile);
	~VideoDecoder();
	
	bool decodeFrame();

	cv::Mat getRotVec() { return rVec; };

	cv::Mat getTrVec() { return tVec; };

	cv::Mat getFrame() { return capturedFrame; };

	std::vector<cv::Mat> getcapturedFrames() { return capturedFrames; };

	cv::VideoCapture* getVidCaptureObject() { return capture; };

	bool decodeGivenFrame(cv::Mat frame);

	int getForucc() { return (int)capture->get(6); };

	bool nextFrame();

	void readAllFrames();

	void release();

private: 

	/* PRIVATE VARIABLES */
	cv::VideoCapture* capture; // webcam's capturer
	cv::Mat capturedFrame; // valid captured frame at the moment
	cv::Mat grayFrame; // grayscale version of the valid captured frame

	std::vector<cv::Mat> capturedFrames;

	cv::Mat camMatrix; // const - a camera-specific matrix, which is given by the camera calibration.
	cv::Mat distCoefMatrix; // const - a camera-specific matrix, which is given by the camera calibration.
	cv::Mat rVec; // calculated rotation vector of the marker object
	cv::Mat tVec; // calculated translation vector of the marker object
	
	std::vector<cv::Point3f> modelPoints; // calculated real positions of the chessboard (in a plane)
	
	cv::Size chessboardDimensions; // dimensions of the chessboard (numbers of the intersections)
	float calibrationSquareDimension; // measured width of each square in the chessboard
	
	/* PRIVATE METHODS */

	void buildCamCalibMatrices(std::string calibResultsPath);

	void createKnownChessboardPoints();
};

