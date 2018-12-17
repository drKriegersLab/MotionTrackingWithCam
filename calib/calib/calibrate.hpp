#ifndef CALIBRATE_H
#define CALIBRATE_H




//void test();


class CameraCalibrator {
public:
	/*
	* Constructor of camera calibration. It initializes the object with the default parameters.
	*/
	CameraCalibrator(bool readFromFile, std::string inputFileName, std::string outputFileName, cv::Size intersectionNum, float chessSquareDim);

	/*
	* Method for performing the calibration
	* - capture the video frame
	* - display the captured image + if the chessboard intersections had been found (the frame is valid) --> draw it
	* - if the frame valid
	*	-- store the frame
	*	-- store the positions of the intersections
	*	-- call OpenCV's calibration, which will provide the translation and rotation matrices
	*/
	void performCalibration();

	/*
	* Destructor
	*/
	~CameraCalibrator();

private:
	// DEFS
	cv::Size chessboardDimensions; // number of crosses in the chessboard
	float calibrationSquareDimension; // [m]: distance between two crosses on the chessboard
	cv::Mat frame; // captured frame from the input
	std::vector<cv::Mat> savedImages; // collection of captured and valid frames
	cv::VideoCapture* videoCapture; // video streamer object
	std::vector<cv::Point2f> foundPoints; // collection of found points in each captured and valid frame
	std::vector<std::vector<cv::Point2f>> checkerboardImageSpacePoints; // found intersection points on the real chessboard
	std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints; // calculated intersection points on the real chessboard
	cv::Mat drawToFrame; // tmp frame for captured and valid frames
	cv::Mat distanceCoefficients; // calculated distance coeffients by OpenCV's calibration method
	cv::Mat cameraMatrix; // calculated distance coeffients by OpenCV's calibration method

	std::string outFilePath; // UNC path of output txt file

	// PRIVATE METHODS
	
	/*
	* method for calculating the real position points of the examined chessboard
	*/
	void createKnownBoardPositions();
	
	/*
	* Method for saving the calibration result to the specified output file as text
	*/
	bool saveCameraCalibration();

};


#endif