#ifndef BEARINGSENSOR_H
#define BEARINGSENSOR_H

#include <sensor.h>
#include <string>
#include <envir.h>
#include <robot.h>
#include <algorithm>
#include <geom.h>

namespace arpro
{
class SensorBearing : public Sensor
{
public:
	SensorBearing (Robot &_robot, double _x, double _y, double _theta):
	Sensor(_robot, _x, _y, _theta)
		{}

void correctTwist(Twist &_v)
{
	cout<<"this is correctTwist function"<<endl;
    _v.w = _v.w - 2 * alpha_;
}
void update(const Pose &_p)
{
    cout<<"this is update function"<<endl;

    for(auto other: envir_ -> robots_)
        if (other != robot_)
        {
            alpha_ = std::atan2(other->pose().y - _p.y, other->pose().x - _p.x) - _p.theta;
            break;
        }
    if(alpha_ < -M_PI)
        alpha_ = alpha_ + 2 * M_PI;
    else if (alpha_ > M_PI)
        alpha_ = alpha_ - 2 * M_PI;
    else
        alpha_ = alpha_;
}

};
}
#endif
