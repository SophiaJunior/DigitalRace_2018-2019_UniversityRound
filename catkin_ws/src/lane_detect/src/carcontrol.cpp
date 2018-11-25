#include "carcontrol.h"

CarController::CarController()
{
    carPos.x = CAR_POSITION_X;
    carPos.y = CAR_POSITION_Y;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>(STEER_ANGLE_PUB,10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>(SPEED_PUB, 10);
}

CarController::~CarController() {}

float CarController::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x) return 0;
    if (dst.y == carPos.y) return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y; 
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void CarController::DriverCar(const vector<Point> &left, const vector<Point> &right, float velocity)
//void CarController::driverCar(const Point &centerLane, float velocity)
{
    int i = left.size() - 11;
    float error = preError;
    while (left[i] == POINT_ZERO && right[i] == POINT_ZERO) {
        i--;
        if (i < 0) return;
    }
    if (left[i] != POINT_ZERO && right[i] != POINT_ZERO)
    {
        error = errorAngle((left[i] + right[i]) / 2);
    } 
    else if (left[i] != POINT_ZERO)
    {
        error = errorAngle(left[i] + Point(laneWidth / 2, 0));
    }
    else
    {
        error = errorAngle(right[i] - Point(laneWidth / 2, 0));
    }
    //float error = errorAngle(centerLane);

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
} 
