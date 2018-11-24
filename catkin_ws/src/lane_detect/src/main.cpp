#include "header.h"
#include "imageprocessing.h"
#include "signdetect.h"
#include "lanedetect.h"
#include "carcontrol.h"

ImageProcessor *ip;
SignDetector *signDetector;
LaneDetector *laneDetector;
CarController *carController;

//VideoWriter *writer;
//VideoCapture *capture;

void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat colorImg, debugImg, hsvImg, grayImg;

    try
    {
        // From subscriber
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        waitKey(1);

        // Original
        colorImg = cv_ptr->image.clone();
        //resize(colorImg, colorImg, FRAME_SIZE);
        imshow("View", colorImg);       

        // Debug
        debugImg = colorImg.clone();
        
        // Log to video
        //writer->write(colorImg);
        
        cvtColor(colorImg, hsvImg, CV_BGR2HSV);
        cvtColor(colorImg, grayImg, CV_BGR2GRAY);
        signDetector->Update(hsvImg, grayImg);

        // if (signDetector->GetType() != ESignType::NONE)
        //     rectangle(debugImg, signDetector->GetRect(), Scalar(0, 255, 0));
        
        laneDetector->Update(colorImg);
        carController->driverCar(laneDetector->getLeftLane(), laneDetector->getRightLane(), 50);
        //laneDetector->Update(cv_ptr->image);
        //carController->driverCar(Point(0, 0), SPEED);

        imshow("Debug", debugImg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void VideoProcess(const char *videoPath)
{
    VideoCapture capture(videoPath);
    
    Mat src;
    while (true)
    {
        capture.read(src);
        if (src.empty())
            break;
        
        imshow("View", src);
        laneDetector->Update(src);
        waitKey(30);
    }
    
    capture.release();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    cv::namedWindow("Binary");
    cv::namedWindow("Bird View");
    cv::namedWindow("Lane Detect");
    cv::namedWindow("Debug");
    cv::namedWindow("Sign Binary");

    SAFE_ALLOC(laneDetector, LaneDetector);
    SAFE_ALLOC(carController, CarController);
    signDetector = new SignDetector("svm_model.xml");
    //SAFE_ALLOC_P1(signDetector, SignDetector, "svm_model.xml");

    //writer = new VideoWriter("out.avi", CV_FOURCC('M','J','P','G'), 30, Size(FRAME_WIDTH, FRAME_HEIGHT));
        
    if (STREAM)
    {
        cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe(IMAGE_SUB, 1, ImageCallback);

        ros::spin();
    }
    else
    {
        VideoProcess("inp.avi");
    }

    //writer->release();

    SAFE_FREE(signDetector);
    SAFE_FREE(laneDetector);
    SAFE_FREE(carController);

    cv::destroyAllWindows();
}
