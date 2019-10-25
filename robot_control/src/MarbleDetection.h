
#ifndef _MARBLEDETECTION_H
#define _MARBLEDETECTION_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

class MarbleDetection
{
    
public:

    MarbleDetection();

    void cameraCallback(ConstImageStampedPtr &msg);
    cv::Mat preprocessing(cv::Mat);
    cv::Mat binaryThreshold(cv::Mat);
    cv::Mat edgeDetection(cv::Mat);
    cv::Mat houghCircles(cv::Mat);

    ~MarbleDetection();

private:



};

#endif