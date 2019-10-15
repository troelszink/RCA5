
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

Fuzzy_control::~Fuzzy_control()
{
}