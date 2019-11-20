
#include "QLearning.h"

QLearning::QLearning()
{
}

void QLearning::map(int resize)
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows));
}

QLearning::~QLearning()
{
}