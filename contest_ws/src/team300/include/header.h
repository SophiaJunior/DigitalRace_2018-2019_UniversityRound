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
#include <algorithm>

// Namespaces
using namespace std;
using namespace cv;
using namespace cv::ml;

#define SAFE_ALLOC(p, T) \
	{                    \
		if (!p)          \
			p = new T(); \
	}
#define SAFE_ALLOC_1P(p, T, param1) \
	{                               \
		if (!p)                     \
			p = new T(param1);      \
	}
#define SAFE_FREE(p)  \
	{                 \
		if (p)        \
			delete p; \
	}

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
#define TEAM_NAME "team300"
#define IMAGE_SUB "team300_image"
#define SPEED_PUB "team300_speed"
#define STEER_ANGLE_PUB "team300_steerAngle"

#define STREAM true
#define USE_ML true
#define SKIP_FRAME 1

#define NORMAL_SPEED 40
#define TURN_SPEED 30
#define SKYLINE 85

#define POINT_ZERO Point(0, 0)

#define RECT_ZERO Rect(0, 0, 0, 0)
#define PI 3.14f

#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define FRAME_SIZE Size(FRAME_WIDTH, FRAME_HEIGHT)

// DetectLane macros
#define SLIDE_THICKNESS 10
#define VERTICAL 0
#define HORIZONTAL 1
#define MARGIN 0.45
#define MORPH_SIZE 1
#define KERNEL_SIZE 3
#define MIN_CONTOUR_AREA 20
#define WINDOW_LANE_WIDTH_RATIO 0.5
#define WINDOW_LANE_HEIGHT_RATIO 0.25
#define CENTERPOINT_HORIZONTAL_RATIO 0.8
#define MARGIN_CENTER_LANE 0.1
#define MIN_AREA_CENTER_LANE 10
#define MAX_AREA_CENTER_LANE 40
#define CENTER_LANE_VERTICAL_RATIO 0.025

#define CENTERPOINT_VERTICAL_TURN_RATIO 0.5 // <= 0.5
#define N_FRAME_TURN 50
#define MIN_CONTOUR_AREA_TURN 500
#define MIN_SIGN_AREA_TO_TURN 200

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
#define LIMIT_DIF_SIGN_AREA 0.5
#define MIN_SIGN_SIZE_PER_FRAME 0.0001f

// Thresholds
#define MIN_HSV_WHITE Scalar(0, 0, 200)
#define MAX_HSV_WHITE Scalar(180, 255, 255)

#define MIN_HSV_BLACK Scalar(0, 0, 0)
#define MAX_HSV_BLACK Scalar(180, 255, 140)

#define MIN_HSV_BLUE Scalar(80, 100, 100)
#define MAX_HSV_BLUE Scalar(120, 255, 255)

#define MIN_SHADOW Scalar(90, 43, 36)
#define MAX_SHADOW Scalar(120, 81, 171)
#define MIN_LANE_SHADOW Scalar(90, 43, 97)
#define MAX_LANE_SHADOW Scalar(120, 80, 171)

enum EColor
{
	BLACK = 0b1,
	WHITE = 0b10,
	BLUE = 0b100
};

enum SignType
{
	NONE = 0,
	TURN_LEFT = 1,
	TURN_RIGHT = 2
};

#define Window_View "View"
#define Window_Debug "Debug"

#define Window_DebugLane "Debug Lane"
#define Window_DebugSign "Debug Sign"

#define Window_BirdBinary "Bird Binary"
#define Window_LaneBinary "Lane Binary"
#define Window_SignBinary "Sign Binary"

#endif
