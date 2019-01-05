#ifndef DETECTLANE_H
#define DETECTLANE_H

#include "header.h"

class LaneDetector
{
  public:
    LaneDetector() = default;
    ~LaneDetector() = default;

  private:
    vector<Point> leftLane, rightLane;
    float laneWidth = LANE_WIDTH;

    Point centerPoint, preCenterPoint;

    vector<Mat> SplitLayer(const Mat &src, int dir = VERTICAL);
    vector<vector<Point>> CenterRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    void DetectLeftRight(const vector<vector<Point>> &points);

    void CalculateCenterPoint();
    void FillLane(Mat &src);
    Mat PreProcess(const Mat &src);

  public:
    void Update(const Mat &src);
    Point GetCenterPoint() { return centerPoint; }
};

#endif
