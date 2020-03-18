#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>
#include <tuple>
#include<cmath>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;

Robot::Robot(string _name, double _x, double _y, double _theta)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);
}

void Robot::initWheel(double r, double b, double w_max)
{
    _b = b;
    _r = r;
    _w_max = w_max;
    wheels_init_attribute = true;
}


void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}



void Robot::rotateWheels(double _left, double _right, double w_max)
{

    // to fill up after defining an initWheel method
    if (wheels_init_attribute == true)
    {

        double a;
        a = std::max((std::abs(_left))/w_max, (std::abs(_right))/w_max);
        if (a < 1)
        {a = 1.0;
            cout << "the velocity of setpoint is too high" << endl;
        }
        _left = _left / a;
        _right = _right / a;



        double _v = _r * (_left + _right)/2;
        double _omega = _r * (_left - _right)/(2 * _b);
        double _vx = _v * cos(pose_.theta);
        double _vy = _v * sin(pose_.theta);
        moveXYT(_vx, _vy, _omega);
    }

}


// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    // to fill up
    double _vx;
    double _vy;
    // pose_.theta += _omega * dt_;
    // _vx = _v * cos(pose_.theta);
    //_vy = _v * sin(pose_.theta);
    // moveXYT(_vx, _vy, _omega);
    double w_l = ( _v + _b * _omega)/_r;
    double w_r = ( _v - _b * _omega)/_r;
    rotateWheels(w_l, w_r, _w_max);
}




// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}


void Robot::moveWithSensor(Twist _twist)
{
    // to fill up, sensor measurement and twist checking
    for (auto &Sensor: sensors_)
    { Sensor->updateFromRobotPose(pose_);

      Sensor->correctRobotTwist(_twist);
    }

    // uses X-Y motion (perfect but impossible in practice)
    //moveXYT(_twist.vx, _twist.vy,_twist.w);

    // to fill up, use V-W motion when defined

    moveVW( _twist.vx, 20 * _twist.vy + _twist.w);
}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

