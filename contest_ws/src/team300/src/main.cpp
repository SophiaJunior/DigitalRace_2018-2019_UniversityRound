#include "header.h"
#include "imageprocessing.h"
#include "signdetect.h"
#include "lanedetect.h"
#include "carcontrol.h"

ImageProcessor *ip;
SignDetector *signDetector;
LaneDetector *laneDetector;
CarController *carController;

void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat colorImg, debugImg;

    try
    {
        // From subscriber
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        waitKey(1);

        // Original
        colorImg = cv_ptr->image.clone();

        imshow(Window_View, colorImg);

        // Sign detect
        signDetector->Update(colorImg);
        if (signDetector->GetType() != SignType::NONE)
        {
            laneDetector->SetSignType(signDetector->GetType());
            laneDetector->SetSignRect(signDetector->GetRect());
        }

        // Lane detect
        laneDetector->Update(colorImg);

        // Control
        carController->SetCarPos(Point(colorImg.cols / 2, colorImg.rows));
        carController->DriverCar(laneDetector->GetCenterPoint());
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");

    cv::namedWindow(Window_View);
    //cv::namedWindow(Window_Debug);
    cv::namedWindow(Window_DebugLane);
    cv::namedWindow(Window_DebugSign);

    cv::namedWindow(Window_BirdBinary);
    cv::namedWindow(Window_LaneBinary);

    SAFE_ALLOC(laneDetector, LaneDetector);
    SAFE_ALLOC(carController, CarController);
    SAFE_ALLOC_1P(signDetector, SignDetector, argv[1]);

    carController->SetVelocity(NORMAL_SPEED);

    if (STREAM)
    {
        // cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe(IMAGE_SUB, 1, ImageCallback);

        ros::spin();
    }

    SAFE_FREE(signDetector);
    SAFE_FREE(laneDetector);
    SAFE_FREE(carController);

    cv::destroyAllWindows();
}
