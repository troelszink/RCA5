
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

void Fuzzy_control::drawPath(std::vector<std::vector<float>> position)
{
    cv::Mat image;
    image = cv::imread("../testImages/SmallWorld.PNG", cv::IMREAD_COLOR);

    for (int i = 0; i < position.size(); i++)
    {
        int x = int(position[i][0] * 72 + 555);
        int y = int(-(position[i][1] * 72) + 380);
        //std::cout << "(" << cb.getVector()[i][0] << "," << cb.getVector()[i][1] << ")" << std::endl;

        cv::circle(image, cv::Point(x,y), 2, cv::Scalar(0, 0, 255), 0, 1, 0);

        /*image.at<cv::Vec3b>(y,x)[0] = 0;
        image.at<cv::Vec3b>(y,x)[1] = 0;
        image.at<cv::Vec3b>(y,x)[2] = 255;*/
    }

    cv::namedWindow("Path", CV_WINDOW_AUTOSIZE);
    cv::imshow("Path", image);

    cv::waitKey();
}

Fuzzy_control::~Fuzzy_control()
{
}