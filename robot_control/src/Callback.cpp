
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

      //    std::cout << angle << " " << range << " " << intensity << std::endl;
      if (range < best_range)
      {
        best_range = range;
        best_angle = angle;

        //std::cout << "angle: " << shortest_angle << "    "  <<  "range: " << shortest_range << std::endl;
      }
            // std::cout << angle << " : " << range << std::endl;
                                                                                            // Angle: radianer, range: antal blokke (nok cm)
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

void Callback::initialize(int _argc, char **_argv)
{

     std::cout << "Starting" << std::endl;
    // Load gazebo 
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber =
        node->Subscribe("~/world_stats", &Callback::statCallback, this);

    gazebo::transport::SubscriberPtr poseSubscriber =
        node->Subscribe("~/pose/info", &Callback::poseCallback, this);

    gazebo::transport::SubscriberPtr cameraSubscriber =
        node->Subscribe("~/pioneer2dx/camera/link/camera/image", &Callback::cameraCallback, this);

    gazebo::transport::SubscriberPtr lidarSubscriber =
        node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &Callback::lidarCallback, this);


    // Publish to the robot vel_cmd topic
    gazebo::transport::PublisherPtr movementPublisher =
        node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher =
        node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);
}

float Callback::getShortestRange()
{
    return shortest_range;
}

float Callback::getShortestAngle()
{
    return shortest_angle;
}

Callback::~Callback()
{
}