
#ifndef _MARBLEDETECTION_H
#define _MARBLEDETECTION_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>

class MarbleDetection
{
    
public:

    MarbleDetection();

    void cameraCallback(ConstImageStampedPtr &msg);

    ~MarbleDetection();

private:



};

#endif