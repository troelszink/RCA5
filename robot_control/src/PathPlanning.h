#ifndef _PATHPLANNING_H
#define _PATHPLANNING_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

#include "Callback.h"

struct cellValue
{
    cv::Point2f p1;
    cv::Point2f p2;
    int value;
};

class PathPlanning
{
    
public:

    PathPlanning();

    void mapIntoCells();
    cv::Mat wavefront();

    ~PathPlanning();

private:



};

#endif