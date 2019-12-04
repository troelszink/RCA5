
#ifndef _CALLBACK_H
#define _CALLBACK_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "fl/Headers.h"

#include <iostream>
#include <vector>

using namespace fl;

class Callback
{

public:

    struct Quaternion
    {
        double w, x, y, z;
    };

    struct EulerAngles
    {
        double roll, pitch, yaw;
    };
    
    Callback(cv::Point2f, float);

    void statCallback(ConstWorldStatisticsPtr &_msg);
    void poseCallback(ConstPosesStampedPtr &_msg);
    void lidarCallback(ConstLaserScanStampedPtr &msg);

    EulerAngles ToEulerAngles(Quaternion);

    float getShortestRange();
    float getShortestAngle();
    float getCornerType();
    cv::Point2f getCurPosition();
    float getYaw();
    bool getFreeLeftPassage();
    bool getFreeRightPassage();
    std::vector<std::vector<float>> getVector();
    std::vector<float> getRangeVector();
    void setSensorIncrement(int);

    ~Callback();
    
protected:

    float shortest_range = 10;
    float shortest_angle = 0;
    float angle_inc;
    float corner_type;
    cv::Point2f curPosition;
    float yaw;
    bool freeLeftPassage;
    bool freeRightPassage;
    std::vector<std::vector<float>> position;
    std::vector<float> rangeVector;
    int sensorIncrement = 1;

    int countIterations = 0;
    float sum = 0;
    float average = 0;

};

#endif
