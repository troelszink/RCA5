
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

    struct Quaternion
    {
        double w, x, y, z;
    };

    struct EulerAngles
    {
        double roll, pitch, yaw;
    };
    
    Callback();

    void statCallback(ConstWorldStatisticsPtr &_msg);
    void poseCallback(ConstPosesStampedPtr &_msg);
    void cameraCallback(ConstImageStampedPtr &msg);
    void lidarCallback(ConstLaserScanStampedPtr &msg);

    EulerAngles ToEulerAngles(Quaternion);

    float getShortestRange();
    float getShortestAngle();
    float getCornerType();
    cv::Point getCurPosition();
    float getYaw();
    bool getFreeLeftPassage();
    bool getFreeRightPassage();

    ~Callback();
    
private:

   
    float shortest_range = 10;
    float shortest_angle = 0;
    float angle_inc;
    float corner_type;
    cv::Point curPosition;
    float yaw;
    bool freeLeftPassage;
    bool freeRightPassage;

};

#endif