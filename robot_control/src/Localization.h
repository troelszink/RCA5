#ifndef _LOCALIZATION_H
#define _LOCALIZATION_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <random>

#include "Callback.h"

struct particle
{
    int id;
    cv::Point2f coord;
    double yaw;
    double weight;
};

class Localization
{

public:

    Localization();

    //void particleFilter();
    //std::vector<particle> generateParticles(int);
    std::vector<double> lidarDistance(cv::Point2f);

    ~Localization();

private:

    int numberOfParticles;

};

#endif