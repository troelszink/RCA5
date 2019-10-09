
#ifndef _CALLBACK_H
#define _CALLBACK_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "fl/Headers.h"

#include <iostream>

using namespace fl;

class Callback
{

public:
    
    Callback();

    void statCallback(ConstWorldStatisticsPtr &_msg);
    void poseCallback(ConstPosesStampedPtr &_msg);
    void cameraCallback(ConstImageStampedPtr &msg);
    void lidarCallback(ConstLaserScanStampedPtr &msg);

    float getShortestRange();
    float getShortestAngle();

    ~Callback();
    
private:

   
    float shortest_range = 10;
    float shortest_angle = 0;
    float angle_inc;

};

#endif