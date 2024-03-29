
#include "Fuzzy_control.h"

Fuzzy_control::Fuzzy_control()
{
}

float Fuzzy_control::normalize(float val, float min, float max)
{
    float norm;
    norm = 2 * ((val - min) / (max - min)) - 1;
    return norm;
}

float Fuzzy_control::distanceToGoal(cv::Point2f curPosition)
{
    float distance = cv::norm(goal - curPosition);
    return distance;
}

float Fuzzy_control::angleToGoal(cv::Point2f curPosition, float curYaw)
{
    float angleHorizontal = atan2(goal.y - curPosition.y, goal.x - curPosition.x); 
    float angle = (angleHorizontal - curYaw);

    if (angle < 0)
    {
        angle = angle + 2*M_PI;
    }

    return angle;// * 180/ 3.14;
}

void Fuzzy_control::drawPathSW(std::vector<std::vector<float>> position)
{
    cv::Mat image;
    image = cv::imread("../testImages/SmallWorldV2.png", cv::IMREAD_COLOR);

    // Only first time
    float resize = 30;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows), 0, 0, cv::INTER_NEAREST);

    cv::Point2f center = cv::Point2f(resize * 10, resize * 7.5);
    float scaling = resize * 1.41735;

    /*float resize = 30;

    cv::Point2f center = cv::Point2f(resize * 10, resize * 7.5);
    float scaling = resize * 1.41735;*/

    for (int i = 0; i < position.size(); i++)
    {
        int x = int(position[i][0] * scaling + center.x);
        int y = int(-position[i][1] * scaling + center.y);;
        cv::circle(image, cv::Point(x,y), 2, cv::Scalar(255, 0, 0), 0, 1, 0);
    }

    cv::imwrite( "../testImages/SmallWorldV2-WF1.png", image );

    cv::namedWindow("Path", CV_WINDOW_AUTOSIZE);
    cv::imshow("Path", image);

    cv::waitKey();
}

void Fuzzy_control::drawPathBW(std::vector<std::vector<float>> position)
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);

    // Only first time
    float resize = 10;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows), 0, 0, cv::INTER_NEAREST);

    cv::Point2f center = cv::Point2f(resize * 60, resize * 40);
    float scaling = resize * 1.41735;

    /*float resize = 10;

    cv::Point2f center = cv::Point2f(resize * 60, resize * 40);
    float scaling = resize * 1.41735;*/

    for (int i = 0; i < position.size(); i++)
    {
        int x = int(position[i][0] * scaling + center.x);
        int y = int(-position[i][1] * scaling + center.y);

        cv::circle(image, cv::Point(x,y), 2, cv::Scalar(0, 0, 255), 0, 1, 0);
    }

    cv::imwrite("../testImages/BigWorldV2-LocalizationPredicted.png", image);

    cv::namedWindow("Path", CV_WINDOW_AUTOSIZE);
    cv::imshow("Path", image);

    cv::waitKey();
}

void Fuzzy_control::setGoal(cv::Point2f _goal)
{
    goal = _goal;
}

Fuzzy_control::~Fuzzy_control()
{
}
