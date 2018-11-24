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

    vector<Mat> splitLayer(const Mat &src, int dir = VERTICAL);
    vector<vector<Point> > centerRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    void detectLeftRight(const vector<vector<Point> > &points);
    Mat laneInShadow(const Mat &src);
    
public:
    void Update(const Mat &src);
    
    vector<Point> getLeftLane();
    vector<Point> getRightLane();
};

#endif
