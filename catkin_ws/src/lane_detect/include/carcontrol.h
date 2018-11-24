#ifndef CARCONTROL_H
#define CARCONTROL_H

#include "header.h"

class CarController 
{
public:
    CarController();
    ~CarController();
    void driverCar(const vector<Point> &left, const vector<Point> &right, float velocity);
    //void driverCar(const Point &centerLane, float velocity);

private:
    float errorAngle(const Point &dst);
    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;
    
    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;

    Point carPos;

    float laneWidth = LANE_WIDTH;

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
