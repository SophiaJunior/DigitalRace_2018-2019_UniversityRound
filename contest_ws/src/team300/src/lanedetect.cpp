#include "carcontrol.h"
#include "lanedetect.h"
#include "signdetect.h"
#include "imageprocessing.h"

void LaneDetector::Update(const Mat &src)
{
    CalculateCenterPoint(src);
    CalculateCenterPoint_Single(src);
}

void LaneDetector::CalculateCenterPoint_Single(const Mat &src)
{
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    Mat birdView = ip->birdViewTranform(gray);

    Rect laneRect = Rect(gray.cols * (0.5 - CENTER_LANE_VERTICAL_RATIO),
                         gray.rows * (1 - WINDOW_LANE_HEIGHT_RATIO),
                         gray.cols * (2 * CENTER_LANE_VERTICAL_RATIO),
                         gray.rows * WINDOW_LANE_HEIGHT_RATIO);

    Mat laneBin = Mat::zeros(gray.rows, gray.cols, CV_8UC1);

    threshold(birdView(laneRect), laneBin(laneRect), 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    // find contours
    vector<Vec4i>
        hierarchy;
    vector<vector<Point>> contours;
    findContours(laneBin, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    int avgX = 0, countAvg = 0;
    for (int iContour = 0; iContour < contours.size(); iContour++)
    {
        double area = contourArea(contours[iContour]);
        // small lane
        if (MIN_CONTOUR_AREA < area && area < MAX_AREA_CENTER_LANE)
        {
            for (Point p : contours[iContour])
                avgX += p.x;
            countAvg += contours[iContour].size();
        }
    }

    if (countAvg && signType == SignType::NONE)
    {
        cout << "center " << countAvg << endl;
        avgX /= countAvg;
        centerX = avgX;
        centerPoint = Point(centerX, gray.rows * CENTERPOINT_HORIZONTAL_RATIO);
    }

    imshow("Lane binary", laneBin);
}

void LaneDetector::CalculateCenterPoint(const Mat &src)
{
    Mat hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    //Mat opened, closed;
    //Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * MORPH_SIZE + 1, 2 * MORPH_SIZE + 1));
    //morphologyEx(bin, opened, MORPH_OPEN, element);
    //morphologyEx(opened, closed, MORPH_CLOSE, element);

    // Threshold
    Mat bin;
    inRange(hsv, MIN_HSV_BLACK, MAX_HSV_BLACK, bin);
    bitwise_not(bin, bin);
    imshow(Window_LaneBinary, bin);

    Mat birdView = ip->birdViewTranform(bin);

    Rect laneRect = Rect(0,
                         bin.rows * (1 - WINDOW_LANE_HEIGHT_RATIO),
                         bin.cols,
                         bin.rows * WINDOW_LANE_HEIGHT_RATIO);

    Mat laneCroped = Mat::zeros(bin.rows, bin.cols, CV_8UC1);
    birdView(laneRect).copyTo(laneCroped(laneRect));

    // find contours
    vector<Vec4i> hierarchy;
    vector<vector<Point>> contours;
    findContours(laneCroped, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    leftX = 0;
    rightX = bin.cols;
    double areaLeft, areaRight = 0;

    for (int iContour = 0; iContour < contours.size(); iContour++)
        if (contourArea(contours[iContour]) > MIN_CONTOUR_AREA)
        {
            drawContours(laneCroped, contours, iContour, Scalar(255), CV_FILLED);

            // Determine if the contour belong to left or right lane
            int avgX = 0;
            int minX = INT_MAX;
            int maxX = INT_MIN;

            for (Point p : contours[iContour])
            {
                avgX += p.x;

                minX = min(minX, p.x);
                maxX = max(maxX, p.x);
            }
            if (contours[iContour].size())
                avgX /= contours[iContour].size();

            double area = contourArea(contours[iContour]);

            // left lane
            if (avgX < laneCroped.cols / 2)
            {
                leftX = max(leftX, maxX);
                areaLeft = max(area, areaLeft);
            }
            // right lane
            else // if (avgX > laneCroped.cols / 2)
            {
                rightX = min(rightX, minX);
                areaRight = max(area, areaRight);
            }
            // center lane
            // if (area < MAX_AREA_CENTER_LANE)
            //     if (laneCroped.cols * (0.5 - MARGIN_CENTER_LANE) < avgX && avgX < laneCroped.cols * (0.5 + MARGIN_CENTER_LANE))
            //     {
            //         countCenterX++;
            //         totalCenterX += avgX;
            //         centerX = totalCenterX / countCenterX;
            //     }
        }

    // if (leftX != 0)
    //     cout << "left ";
    // if (rightX != bin.cols)
    //     cout << "right ";
    // cout << endl;

    Mat laneDebug;
    cvtColor(laneCroped, laneDebug, COLOR_GRAY2BGR);

    circle(laneDebug, Point(leftX, bin.rows), 5, Scalar(0, 255, 0));
    circle(laneDebug, Point(rightX, bin.rows), 5, Scalar(0, 255, 0));

    if (leftX == 0 || rightX == bin.cols)
    {
        leftX = preLeftX;
        rightX = preRightX;
    }
    else
    {
        preLeftX = leftX;
        preRightX = rightX;
    }
    cout << leftX << ' ' << rightX << endl;

    // Turn cases
    if (nFrameTurn >= N_FRAME_TURN)
        signType = SignType::NONE;

    // if (signType != SignType::NONE)
    //     cout << "Sign area" << signRect.area() << endl;

    if (signRect.area() > MIN_SIGN_AREA_TO_TURN && signType == SignType::TURN_LEFT)
    {
        nFrameTurn++;
        carController->SetVelocity(TURN_SPEED);
        centerPoint = Point(leftX + laneWidth / 2, bin.rows * CENTERPOINT_HORIZONTAL_RATIO);
    }
    else if (signRect.area() > MIN_SIGN_AREA_TO_TURN && signType == SignType::TURN_RIGHT)
    {
        nFrameTurn++;
        carController->SetVelocity(TURN_SPEED);
        centerPoint = Point(rightX - laneWidth / 2, bin.rows * CENTERPOINT_HORIZONTAL_RATIO);
    }
    else
    {
        nFrameTurn = 0;
        carController->SetVelocity(NORMAL_SPEED);
        centerPoint = Point((leftX + rightX) / 2, bin.rows * CENTERPOINT_HORIZONTAL_RATIO);

        if (leftX != 0 && rightX != bin.cols)
        {
            countTimesLaneWidth++;
            totalLaneWidth += abs(leftX - rightX);
            laneWidth = totalLaneWidth / countTimesLaneWidth;
        }
    }
    line(laneDebug, centerPoint, Point(bin.cols / 2, bin.rows), Scalar(255, 0, 0), 3);
    imshow(Window_DebugLane, laneDebug);
}
