#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include "header.h"

class CarController
{
  public:
    CarController();
    ~CarController();
    void DriverCar(Point centerPoint, float velocity);

  private:
    float errorAngle(const Point &dst);
    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;

    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;

    Point carPos;

    float minVelocity = MIN_VELOCITY;
    float maxVelocity = MAX_VELOCITY;

    float preError;

    float kP;
    float kI;
    float kD;

    int t_kP;
    int t_kI;
    int t_kD;
};

#endif
