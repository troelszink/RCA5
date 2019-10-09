
#include "Fuzzy_control.h"

Fuzzy_control::Fuzzy_control()
{
}

float Fuzzy_control::normalize(float val, std::string type)
{
    if (type == "range")
    {
        float range_min = 0.08;
        float range_max = 10;
        float norm_range;

        norm_range = 2 * ((val - range_min) / (range_max - range_min)) - 1;
        return norm_range;
    }
    else if (type == "angle")
    {
        float angle_min = 2.26889;
        float angle_max = -2.2689;
        float norm_angle;

        norm_angle = 2 * ((val - angle_min) / (angle_max - angle_min)) - 1;
        return norm_angle;
    }
}

Fuzzy_control::~Fuzzy_control()
{
}