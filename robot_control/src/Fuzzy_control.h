
#ifndef _FUZZY_CONTROL_H
#define _FUZZY_CONTROL_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "fl/Headers.h"

#include <iostream>

using namespace fl;

class Fuzzy_control
{

public:

    Fuzzy_control();

    float normalize(float val, std::string type);

    ~Fuzzy_control();

private:

};

#endif