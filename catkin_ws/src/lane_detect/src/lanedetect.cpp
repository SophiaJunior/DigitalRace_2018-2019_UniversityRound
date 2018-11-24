#include "lanedetect.h"
#include "imageprocessing.h"

vector<Point> LaneDetector::getLeftLane()
{
    return leftLane;
}

vector<Point> LaneDetector::getRightLane()
{
    return rightLane;
}

void LaneDetector::Update(const Mat &src)
{
    //ip->Binarialize();
    Mat img = ip->PreProcess(src);
    
    vector<Mat> layers1 = splitLayer(img);
    vector<vector<Point> > points1 = centerRoadSide(layers1);
    // vector<Mat> layers2 = splitLayer(img, HORIZONTAL);
    // vector<vector<Point> > points2 = centerRoadSide(layers2, HORIZONTAL);

    detectLeftRight(points1);

    Mat birdView, lane;
    birdView = Mat::zeros(img.size(), CV_8UC3);
    lane = Mat::zeros(img.size(), CV_8UC3);

    for (int i = 0; i < points1.size(); i++)
     {
        for (int j = 0; j < points1[i].size(); j++)
        {
            circle(birdView, points1[i][j], 1, Scalar(0,0,255), 2, 8, 0 );
        }
    }

    // for (int i = 0; i < points2.size(); i++)
    //  {
    //     for (int j = 0; j < points2[i].size(); j++)
    //     {
    //         circle(birdView, points2[i][j], 1, Scalar(0,255,0), 2, 8, 0 );
    //     }
    // }

    // imshow("Debug", birdView);

    for (int i = 1; i < leftLane.size(); i++)
    {
        if (leftLane[i] != POINT_ZERO)
        {
            circle(lane, leftLane[i], 1, Scalar(0,0,255), 2, 8, 0 );
        }
    }

    for (int i = 1; i < rightLane.size(); i++)
    {
        if (rightLane[i] != POINT_ZERO) {
            circle(lane, rightLane[i], 1, Scalar(255,0,0), 2, 8, 0 );
        }
    }

    imshow("Lane Detect", lane);
}

Mat LaneDetector::laneInShadow(const Mat &src)
{
    Mat shadowMask, shadow, imgHSV, shadowHSV, laneShadow;
    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, MIN_SHADOW, MAX_SHADOW, shadowMask);

    src.copyTo(shadow, shadowMask);

    cvtColor(shadow, shadowHSV, COLOR_BGR2HSV);

    inRange(shadowHSV, MIN_LANE_SHADOW, MAX_LANE_SHADOW, laneShadow);

    return laneShadow;
}

vector<Mat> LaneDetector::splitLayer(const Mat &src, int dir)
{
    int rowN = src.rows;
    int colN = src.cols;
    std::vector<Mat> res;

    if (dir == VERTICAL)
    {
        for (int i = 0; i < rowN - SLIDE_THICKNESS; i += SLIDE_THICKNESS) {
            Mat tmp;
            Rect crop(0, i, colN, SLIDE_THICKNESS);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    else 
    {
        for (int i = 0; i < colN - SLIDE_THICKNESS; i += SLIDE_THICKNESS) {
            Mat tmp;
            Rect crop(i, 0, SLIDE_THICKNESS, rowN);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    
    return res;
}

vector<vector<Point> > LaneDetector::centerRoadSide(const vector<Mat> &src, int dir)
{
    vector<std::vector<Point> > res;
    int inputN = src.size();
    for (int i = 0; i < inputN; i++) {
        std::vector<std::vector<Point> > cnts;
        std::vector<Point> tmp;
        findContours(src[i], cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        int cntsN = cnts.size();
        if (cntsN == 0) {
            res.push_back(tmp);
            continue;
        }

        for (int j = 0; j < cntsN; j++) {
            int area = contourArea(cnts[j], false);
            if (area > 3) {
                Moments M1 = moments(cnts[j], false);
                Point2f center1 = Point2f(static_cast<float> (M1.m10 / M1.m00), static_cast<float> (M1.m01 / M1.m00));
                if (dir == VERTICAL) {
                    center1.y = center1.y + SLIDE_THICKNESS * i;
                } 
                else
                {
                    center1.x = center1.x + SLIDE_THICKNESS * i;
                }
                if (center1.x > 0 && center1.y > 0) {
                    tmp.push_back(center1);
                }
            }
        }
        res.push_back(tmp);
    }

    return res;
}

void LaneDetector::detectLeftRight(const vector<vector<Point> > &points)
{
    static vector<Point> lane1, lane2;
    lane1.clear();
    lane2.clear();
    
    leftLane.clear();
    rightLane.clear();
    for (int i = 0; i < BIRDVIEW_HEIGHT / SLIDE_THICKNESS; i ++)
    {
        leftLane.push_back(POINT_ZERO);
        rightLane.push_back(POINT_ZERO);
    }

    int pointMap[points.size()][20];
    int prePoint[points.size()][20];
    int postPoint[points.size()][20];
    int dis = 10;
    int max = -1, max2 = -1;
    Point2i posMax, posMax2;

    memset(pointMap, 0, sizeof pointMap);

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }

    for (int i = points.size() - 2; i >= 0; i--)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            int err = 320;
            for (int m = 1; m < min((int)points.size() - 1 - i, 5); m++)
            {
                bool check = false;
                for (int k = 0; k < points[i + 1].size(); k ++)
                {
                    if (abs(points[i + m][k].x - points[i][j].x) < dis && 
                    abs(points[i + m][k].x - points[i][j].x) < err) {
                        err = abs(points[i + m][k].x - points[i][j].x);
                        pointMap[i][j] = pointMap[i + m][k] + 1;
                        prePoint[i][j] = k;
                        postPoint[i + m][k] = j;
                        check = true;
                    }
                }   
                break; 
            }
            
            if (pointMap[i][j] > max) 
            {
                max = pointMap[i][j];
                posMax = Point2i(i, j);
            }
        }
    }

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max2 && (i != posMax.x || j != posMax.y) && postPoint[i][j] == -1)
            {
                max2 = pointMap[i][j];
                posMax2 = Point2i(i, j);
            }
        }
    }

    if (max == -1) return;

    while (max >= 1)
    {
        lane1.push_back(points[posMax.x][posMax.y]);
        if (max == 1) break;

        posMax.y = prePoint[posMax.x][posMax.y];
        posMax.x += 1;        
        
        max--;
    }

    while (max2 >= 1)
    {
        lane2.push_back(points[posMax2.x][posMax2.y]);
        if (max2 == 1) break;

        posMax2.y = prePoint[posMax2.x][posMax2.y];
        posMax2.x += 1;        
        
        max2--;
    }
    
    vector<Point> subLane1(lane1.begin(), lane1.begin() + 5);
    vector<Point> subLane2(lane2.begin(), lane2.begin() + 5);

    Vec4f line1, line2;

    fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
    fitLine(subLane2, line2, 2, 0, 0.01, 0.01);

    int lane1X = (BIRDVIEW_WIDTH - line1[3]) * line1[0] / line1[1] + line1[2];
    int lane2X = (BIRDVIEW_WIDTH - line2[3]) * line2[0] / line2[1] + line2[2];

    if (lane1X < lane2X)
    {
        for (int i = 0; i < lane1.size(); i++)
        {
            leftLane[floor(lane1[i].y / SLIDE_THICKNESS)] = lane1[i];
        }
        for (int i = 0; i < lane2.size(); i++)
        {
            rightLane[floor(lane2[i].y / SLIDE_THICKNESS)] = lane2[i];
        }
    }
    else
    {
        for (int i = 0; i < lane2.size(); i++)
        {
            leftLane[floor(lane2[i].y / SLIDE_THICKNESS)] = lane2[i];
        }
        for (int i = 0; i < lane1.size(); i++)
        {
            rightLane[floor(lane1[i].y / SLIDE_THICKNESS)] = lane1[i];
        }
    }
}