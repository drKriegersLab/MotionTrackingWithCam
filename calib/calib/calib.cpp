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

int main()
{
	string outputFile = "muhaha";

	CameraCalibrator cameraCalibrator(outputFile);
	cameraCalibrator.performCalibration();

	return 0;
    
}
