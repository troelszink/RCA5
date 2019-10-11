
#include "Callback.h"

Callback::Callback()
{
   
}

void Callback::statCallback(ConstWorldStatisticsPtr &_msg) 
{
    (void)_msg;
    // Dump the message contents to stdout.
    //  std::cout << _msg->DebugString();
    //  std::cout << std::flush;
}

void Callback::poseCallback(ConstPosesStampedPtr &_msg) 
{
    // Dump the message contents to stdout.
    //  std::cout << _msg->DebugString();

    for (int i = 0; i < _msg->pose_size(); i++) 
    {
        if (_msg->pose(i).name() == "pioneer2dx") 
        {

        /* std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                    << _msg->pose(i).position().x() << std::setw(6)
                    << _msg->pose(i).position().y() << std::setw(6)
                    << _msg->pose(i).position().z() << std::setw(6)
                    << _msg->pose(i).orientation().w() << std::setw(6)
                    << _msg->pose(i).orientation().x() << std::setw(6)
                    << _msg->pose(i).orientation().y() << std::setw(6)
                    << _msg->pose(i).orientation().z() << std::endl;*/
        }
    }
}

void Callback::cameraCallback(ConstImageStampedPtr &msg) 
{
      static boost::mutex mutex;

    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    im = im.clone();
    cv::cvtColor(im, im, CV_RGB2BGR);

    mutex.lock();
    cv::imshow("camera", im);
    mutex.unlock();
}

void Callback::lidarCallback(ConstLaserScanStampedPtr &msg) 
{
      static boost::mutex mutex;

    //  std::cout << ">> " << msg->DebugString() << std::endl;
    float angle_min = float(msg->scan().angle_min());
    //  double angle_max = msg->scan().angle_max();
    float angle_increment = float(msg->scan().angle_step());


    angle_inc = angle_increment;                      //Test

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


    for (int i = 0; i < nranges; i++) {
      float angle = angle_min + i * angle_increment;
      float range = std::min(float(msg->scan().ranges(i)), range_max);             
      //    double intensity = msg->scan().intensities(i);
      cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                          200.5f - range_min * px_per_m * std::sin(angle));
      cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                        200.5f - range * px_per_m * std::sin(angle));
      cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
              cv::LINE_AA, 4);

      if (range < best_range && 66 <= i <= nranges - 66)
      {
        best_range = range;
        best_angle = angle;

      }
      if (i == 33)
        right_range = range;
      if (i == nranges - 33)
        left_range = range;
                                                                       // Angle: radianer, range: antal blokke (nok cm)
    }

    if (right_range < left_range)
        corner_type = 1;
    else
        corner_type = -1;

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

Callback::~Callback()
{
}