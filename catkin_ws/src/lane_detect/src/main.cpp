#include "header.h"
#include "imageprocessing.h"
#include "signdetect.h"
#include "lanedetect.h"
#include "carcontrol.h"

ImageProcessor *ip;
SignDetector *signDetector;
LaneDetector *laneDetector;
CarController *carController;

Mat debugImg;

//VideoWriter *writer;
//VideoCapture *capture;

void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat colorImg;

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
        
        signDetector->Update(colorImg);

        ESignType signType = signDetector->GetType();
        Rect signRect = signDetector->GetRect();

        if (signType != ESignType::NONE)
        {
            cout << (signType == ESignType::TURN_LEFT ? "Turn left" : "Turn right") << endl;

            rectangle(debugImg, signRect, Scalar(0, 0, 255));
            putText(debugImg, 
                (signType == ESignType::TURN_LEFT ? "Turn left" : "Turn right"), 
                Point(signRect.x, signRect.y),
                CV_FONT_HERSHEY_COMPLEX_SMALL,
                0.5,
                Scalar(0, 0, 255)
            );
        }

        laneDetector->Update(colorImg);
        carController->DriverCar(laneDetector->getLeftLane(), laneDetector->getRightLane(), SPEED);
        
        imshow("Debug", debugImg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void ImageProcess(const char *path)
{
    Mat src = imread(path);
    imshow("View", src);
    //laneDetector->Update(src);
    waitKey(0);
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
    //cv::namedWindow("crop");

    SAFE_ALLOC(laneDetector, LaneDetector);
    SAFE_ALLOC(carController, CarController);

    //signDetector = new SignDetector("svm_model.xml");
    SAFE_ALLOC_P1(signDetector, SignDetector, "svm_model.xml");

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
        //ImageProcess("test.png");
        //VideoProcess("inp.avi");
    }

    //writer->release();

    SAFE_FREE(signDetector);
    SAFE_FREE(laneDetector);
    SAFE_FREE(carController);

    cv::destroyAllWindows();
}
