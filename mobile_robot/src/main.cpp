#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensor_range.h>
#include <sensor_bearing.h>
#include <tuple>

using namespace std;
using namespace arpro;

int main(int argc, char **argv)
{

  // default environment with moving target
  Environment envir;
  // sensors gets measurements from this environment
  Sensor::setEnvironment(envir);

  // init robot at (0,0,0)
  Robot robot("R2D2", 0,0,0);
  Robot robot2("R2D2_2", 0, 0, 0);
  RangeSensor rangesensor(robot, 0.1, 0, 0);
  SensorBearing bearingsensor(robot2, 0.1, 0, 0);

  envir.addRobot(robot);
  envir.addRobot(robot2);
  robot.initWheel(0.07, 0.3, 10);
  robot2.initWheel(0.05, 0.3, 10);



  // simulate 100 sec
  while(envir.time() < 100)
  {
    cout << "---------------------" << endl;

    robot2.moveWithSensor(Twist(0.4, 0, 0));
    robot2.printPosition();

    // update target position
    envir.updateTarget();

    // try to follow target
    robot.goTo(envir.target());


  }

  // plot trajectory
  envir.plot();


}
