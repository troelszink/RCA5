
#ifndef _MARBLEDETECTION_H
#define _MARBLEDETECTION_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

#include "Callback.h"

class MarbleDetection
{
    
public:

    MarbleDetection();

    void cameraCallback(ConstImageStampedPtr &msg);
    cv::Mat preprocessing(cv::Mat);
    cv::Mat binaryThreshold(cv::Mat);
    cv::Mat edgeDetection(cv::Mat);
    cv::Mat houghCircles(cv::Mat);

    void addObject(Callback&);
    void marbleLocation(float, float, float);
    void drawTest();

    ~MarbleDetection();

private:

    float focalLength = 277.559; // See Mathematica
    float marbleRadius = 0.5;
    float cameraWidth = 320;
    float FOV = 1.047;
    Callback* callback;
    int countDiameter = 0;
    int sumDiameter = 0;
    int sumX = 0;
    int sumY = 0;

};

#endif