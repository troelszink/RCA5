
#ifndef _FUZZY_CONTROL_H
#define _FUZZY_CONTROL_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "fl/Headers.h"

#include <iostream>
#include <math.h>

using namespace fl;

class Fuzzy_control
{

public:

    Fuzzy_control();

    float normalize(float val, float min, float max);
    float distanceToGoal(cv::Point);
    float angleToGoal(cv::Point, float);

    ~Fuzzy_control();

private:

    cv::Point goal = cv::Point(-4, 2);

};

#endif