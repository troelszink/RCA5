
#include "Callback.h"

Callback::Callback(cv::Point2f _curPosition, float _yaw)
{
    curPosition = _curPosition;
    yaw = _yaw;
}

void Callback::statCallback(ConstWorldStatisticsPtr &_msg) 
{
    (void)_msg;
}

void Callback::poseCallback(ConstPosesStampedPtr &_msg) 
{
    for (int i = 0; i < _msg->pose_size(); i++) 
    {
        if (_msg->pose(i).name() == "pioneer2dx") 
        {
            Quaternion q;

            q.w = _msg->pose(i).orientation().w();
            q.x = _msg->pose(i).orientation().x();
            q.y = _msg->pose(i).orientation().y();
            q.z = _msg->pose(i).orientation().z();

            ToEulerAngles(q);

            yaw = ToEulerAngles(q).yaw;

            curPosition.x = _msg->pose(i).position().x();
            curPosition.y = _msg->pose(i).position().y();

            // Storing the current position in a vector
            std::vector<float> points;
            points.push_back(_msg->pose(i).position().x());
            points.push_back(_msg->pose(i).position().y());
            position.push_back(points);
        }
    }
}

void Callback::lidarCallback(ConstLaserScanStampedPtr &msg) 
{
      static boost::mutex mutex;

    float angle_min = float(msg->scan().angle_min());
    float angle_increment = float(msg->scan().angle_step());

    angle_inc = angle_increment;

    float range_min = float(msg->scan().range_min());
    float range_max = float(msg->scan().range_max());

    int sec = msg->time().sec();
    int nsec = msg->time().nsec();

    int nranges = msg->scan().ranges_size();
 
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);

    int width = 400;
    int height = 400;
    float px_per_m = 200 / range_max;

    cv::Mat im(height, width, CV_8UC3);
    im.setTo(0);
   
    float best_range = range_max;
    float best_angle = angle_min;

    float left_range;
    float right_range;
    int best_i;

    float leftPassageVal = 0;
    float rightPassageVal = 0;

    std::vector<float> rangeVec;

    for (int i = 0; i < nranges; i+=sensorIncrement)
    {
        float angle = angle_min + i * angle_increment;
        float range = std::min(float(msg->scan().ranges(i)), range_max);
        rangeVec.push_back(range);
        cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                            200.5f - range_min * px_per_m * std::sin(angle));
        cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                            200.5f - range * px_per_m * std::sin(angle));
        cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
                cv::LINE_AA, 4);

        if (range < best_range && i >= 66 && i <= nranges - 66)
        {
            best_range = range;
            best_angle = angle;
            best_i = i;
        }

        if (i == 33)
        {
            right_range = range;
        }
        else if (i == (nranges - 33))
        {
            left_range = range;
        }

        if (right_range < left_range)
        {
            corner_type = 1;
        }
        else
        {
            corner_type = -1;
        }          
        
        if ( i >= 33 - 7 && i <= 33 + 7 ) // right
        {
            rightPassageVal += range;
        }
        else if ( i >= nranges - 33 - 7 && i <= nranges - 33 + 7 ) // left
        {
            leftPassageVal += range;
        }        

        // Middle sensor (to calculate distance from robot to marble)
        if (i == nranges/2)
        {
            countIterations++;
            sum += range;

            if (countIterations == 100)
            {
                average = sum / 100;
                countIterations = 0;
                sum = 0;
            }
        }                                                                          
    }

    rangeVector = rangeVec;
    
    if ( rightPassageVal > (15 * 2) ) // sensors times distance
    {
        freeRightPassage = true;
    }
    else
    {
        freeRightPassage = false;
    }
    
    if ( leftPassageVal > (15 * 2) )
    {
        freeLeftPassage = true;
    }
    else
    {
        freeLeftPassage = false;
    }

    mutex.lock();
    shortest_range = best_range;
    shortest_angle = best_angle;
    mutex.unlock();
  
    cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
                cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(255, 0, 0));

    mutex.lock();
    cv::imshow("lidar", im);
    mutex.unlock();
}

Callback::EulerAngles Callback::ToEulerAngles(Quaternion q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
    {
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    }
    else
    {
        angles.pitch = asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}

float Callback::getShortestRange()
{
    return shortest_range;
}

float Callback::getShortestAngle()
{
    return shortest_angle;
}

float Callback::getCornerType()
{
    return corner_type;
}

cv::Point2f Callback:: getCurPosition()
{
    return curPosition;
}

float Callback::getYaw()
{
    return yaw;
}

bool Callback::getFreeLeftPassage()
{
    return freeLeftPassage;
}

bool Callback::getFreeRightPassage()
{
    return freeRightPassage;
}

std::vector<std::vector<float>> Callback::getVector()
{
    return position;
}

std::vector<float> Callback::getRangeVector()
{
    return rangeVector;
}

void Callback::setSensorIncrement(int _sensorIncrement)
{
    sensorIncrement = _sensorIncrement;
}

Callback::~Callback()
{
}
