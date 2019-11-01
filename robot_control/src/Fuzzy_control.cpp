
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

float Fuzzy_control::distanceToGoal(cv::Point curPosition)
{
    float distance = cv::norm(goal - curPosition);

    return distance;
}

float Fuzzy_control::angleToGoal(cv::Point curPosition, float curYaw)
{
    /*float dotProduct = (curPosition.x * goal.x) + (curPosition.y * goal.y);
    float curPosLen = sqrt(pow(curPosition.x, 2) + pow(curPosition.y, 2));
    float goalLen = sqrt(pow(goal.x, 2) + pow(goal.y, 2));

    float angle = acos((dotProduct) / (curPosLen * goalLen));*/

    float angleHorizontal = atan2(goal.y - curPosition.y, goal.x - curPosition.x); 

    float angle = (angleHorizontal - curYaw);

    if (angle < 0)
        angle = angle + 2*M_PI;

    return angle;// * 180/ 3.14;
}

void Fuzzy_control::drawPathSW(std::vector<std::vector<float>> position)
{
    cv::Mat image;
    image = cv::imread("../testImages/SmallWorldV2.png", cv::IMREAD_COLOR);

    float resize = 5;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows), 0, 0, CV_INTER_LINEAR);

    cv::Point2f center = cv::Point2f(resize * 10, resize * 7.5);
    float scaling = resize * 1.41735;

    for (int i = 0; i < position.size(); i++)
    {
        // 72 is length and width of a field in the world
        // (555,380) is the origo of the image, which is (0,0) in the world
        int x = int(position[i][0] * scaling + center.x);
        int y = int(-position[i][1] * scaling + center.y);;
        //std::cout << "(" << cb.getVector()[i][0] << "," << cb.getVector()[i][1] << ")" << std::endl;

        cv::circle(image, cv::Point(x,y), 2, cv::Scalar(0, 0, 255), 0, 1, 0);

        /*image.at<cv::Vec3b>(y,x)[0] = 0;
        image.at<cv::Vec3b>(y,x)[1] = 0;
        image.at<cv::Vec3b>(y,x)[2] = 255;*/
    }

    cv::imwrite( "../testImages/SmallWorldV2-1.png", image );

    cv::namedWindow("Path", CV_WINDOW_AUTOSIZE);
    cv::imshow("Path", image);

    cv::waitKey();
}

void Fuzzy_control::drawPathBW(std::vector<std::vector<float>> position)
{
    cv::Mat image;
    image = cv::imread("../testImages/BigWorldV2.png", cv::IMREAD_COLOR);

    float resize = 5;
    cv::resize(image, image, cv::Size(resize*image.cols, resize*image.rows));

    cv::Point2f center = cv::Point2f(resize * 60, resize * 40);
    float scaling = resize * 1.41735;

    for (int i = 0; i < position.size(); i++)
    {
        // 72 is length and width of a field in the world
        // (555,380) is the origo of the image, which is (0,0) in the world
        int x = int(position[i][0] * scaling + center.x);
        int y = int(-position[i][1] * scaling + center.y);
        //std::cout << "(" << cb.getVector()[i][0] << "," << cb.getVector()[i][1] << ")" << std::endl;

        cv::circle(image, cv::Point(x,y), 2, cv::Scalar(0, 0, 255), 0, 1, 0);

        /*image.at<cv::Vec3b>(y,x)[0] = 0;
        image.at<cv::Vec3b>(y,x)[1] = 0;
        image.at<cv::Vec3b>(y,x)[2] = 255;*/
    }

    cv::imwrite("../testImages/BigWorldV2-1.png", image);

    cv::namedWindow("Path", CV_WINDOW_AUTOSIZE);
    cv::imshow("Path", image);

    cv::waitKey();
}

Fuzzy_control::~Fuzzy_control()
{
}