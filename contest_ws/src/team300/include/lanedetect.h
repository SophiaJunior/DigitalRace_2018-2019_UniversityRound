#ifndef DETECTLANE_H
#define DETECTLANE_H

#include "header.h"

class LaneDetector
{
public:
  LaneDetector()
  {
    signType = SignType::NONE;
    isTurning = false;
    nFrameTurn = 0;
    countTimesLaneWidth = totalLaneWidth = 0;
  }
  ~LaneDetector() = default;

private:
  Point centerPoint;
  int leftX, rightX;
  int preLeftX, preRightX;
  int centerX;

  SignType signType;
  Rect signRect;

  bool isTurning;
  int nFrameTurn;

  float laneWidth;
  int countTimesLaneWidth;
  int totalLaneWidth;

  void CalculateCenterPoint_Single(const Mat &src);
  void CalculateCenterPoint(const Mat &src);

public:
  void Update(const Mat &src);
  Point GetCenterPoint() { return centerPoint; }

  void SetSignType(SignType t) { signType = t; }
  void SetSignRect(Rect r) { signRect = r; }
};

#endif
