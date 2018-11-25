#ifndef HEADER_H
#define HEADER_H

// External libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>


#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// C++
#include <math.h>
#include <vector>
#include <algorithm>
#include <fstream>

// Namespaces
using namespace std;
using namespace cv;
using namespace cv::ml;

#define SAFE_ALLOC(p, T) { if (!p) p = new T(); }
#define SAFE_ALLOC_P1(p, T, param1) { if (!p) p = new T(param1); }
#define SAFE_FREE(p) { if (p) delete p; }

// Pre-definition
class ImageProcessor;
class LaneDetector;
class CarController;
class SignDetector;

// Global variables
extern ImageProcessor *ip;
extern LaneDetector *laneDetector;
extern CarController *carController;
extern SignDetector *signDetector;
extern Mat debugImg;

// macros
#define TEAM_NAME "Team1"
#define IMAGE_SUB "Team1_image"
#define SPEED_PUB "Team1_speed"
#define STEER_ANGLE_PUB "Team1_steerAngle"

#define STREAM true
#define USE_ML false
#define SKIP_FRAME 1

#define SPEED 50
#define SKYLINE 85

#define POINT_ZERO Point(0, 0)

#define RECT_ZERO Rect(0, 0, 0, 0)
#define PI 3.14f

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define FRAME_SIZE Size(FRAME_WIDTH, FRAME_HEIGHT)

// DetectLane macros
#define SLIDE_THICKNESS 10
#define BIRDVIEW_WIDTH 240
#define BIRDVIEW_HEIGHT 320
#define VERTICAL 0
#define HORIZONTAL 1

// CarControl Macros
#define LANE_WIDTH 40
#define MIN_VELOCITY 10
#define MAX_VELOCITY 50
#define CAR_POSITION_X 120
#define CAR_POSITION_Y 300

// Sign macros
#define SIGN_WIDTH 32
#define SIGN_HEIGHT 32
#define SIGN_SIZE Size(SIGN_WIDTH, SIGN_HEIGHT)

#define LIMIT_DIF_SIGN_SIZE 0.5
#define LIMIT_DIF_SIGN_AREA 0.4
#define MIN_SIGN_SIZE_PER_FRAME 0.001f

// Thresholds
#define MIN_HSV_BLACK Scalar(0, 0, 180)
#define MAX_HSV_BLACK Scalar(179, 30, 255)

#define MIN_HSV_BLUE Scalar(80, 100, 100)
#define MAX_HSV_BLUE Scalar(120, 255, 255)

#define MIN_SHADOW Scalar(90, 43, 36)
#define MAX_SHADOW Scalar(120, 81, 171)
#define MIN_LANE_SHADOW Scalar(90, 43, 97)
#define MAX_LANE_SHADOW Scalar(120, 80, 171)

//#define SHADOW_PARAM 40
//#define BINARY_THRESH 180

enum EColor
{
	BLACK = 0b1,
	WHITE = 0b10,
	BLUE = 0b100	
};

enum ESignType
{
	NONE = 0,
	TURN_LEFT = 1,
	TURN_RIGHT = 2
};

#endif
