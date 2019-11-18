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
    int value = 0;
};

class PathPlanning
{
    
public:

    PathPlanning();

    void mapIntoCells();
    std::vector<cellValue> wavefront();
    cv::Point2f robotControl(std::vector<cellValue>, cv::Point2f);
    cv::Point2f getGoal();
    std::vector<cellValue> brushfire();
    bool isOuterWall(int, std::vector<cellValue>);
    bool isValidRoom(int, std::vector<cellValue>);

    ~PathPlanning();

private:

    cv::Point2f start = cv::Point2f(0, 0);
    cv::Point2f goal = cv::Point2f(5, -4);

};

#endif