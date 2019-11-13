
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
    float distanceToGoal(cv::Point2f);
    float angleToGoal(cv::Point2f, float);
    void drawPathSW(std::vector<std::vector<float>>);
    void drawPathBW(std::vector<std::vector<float>>);
    void setGoal(cv::Point2f);

    ~Fuzzy_control();

private:

    cv::Point2f goal = cv::Point2f(0, 0);

};

#endif