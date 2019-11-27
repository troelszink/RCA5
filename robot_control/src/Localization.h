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
#include <fstream>

#include "Callback.h"

struct particle
{
    int id;
    cv::Point2f coord;
    float yaw;
    float weight;
    std::vector<float> lidarData;
};

class Localization
{

public:

    Localization();

    cv::Point2f localize(std::vector<float>);
    std::vector<particle> generateParticles(int);
    std::vector<float> lidarDistance(cv::Point2f, float);
    std::vector<particle> updateWeigths(std::vector<particle>, std::vector<float>, cv::Point2f);
    std::vector<particle> resample(std::vector<particle>);
    void displayParticles(std::vector<particle>);
    void saveCoords(std::vector<particle>, cv::Point2f);
    void createCSVfile(std::vector<std::vector<float>>);

    ~Localization();

private:

    int numberOfParticles;
    int nCoordinates = 0;
    std::vector<std::vector<float>> coordinatesVector;

};

#endif