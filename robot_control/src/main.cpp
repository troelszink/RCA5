#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "fl/Headers.h"

#include <iostream>

// Classes

#include "Fuzzy_control.h"
#include "Callback.h"

using namespace fl;

// Global variables

int main(int _argc, char **_argv) 
{
    static boost::mutex mutex;

    // Create objects
    Callback cb;
    Fuzzy_control fc;

    std::cout << "Starting" << std::endl;
    // Load gazebo 
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber =
        node->Subscribe("~/world_stats", &Callback::statCallback, &cb);

    gazebo::transport::SubscriberPtr poseSubscriber =
        node->Subscribe("~/pose/info", &Callback::poseCallback, &cb);

    gazebo::transport::SubscriberPtr cameraSubscriber =
        node->Subscribe("~/pioneer2dx/camera/link/camera/image", &Callback::cameraCallback, &cb);

    gazebo::transport::SubscriberPtr lidarSubscriber =
        node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &Callback::lidarCallback, &cb);


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

    

    const int key_left = 81;
    const int key_up = 82;
    const int key_down = 84;
    const int key_right = 83;
    const int key_esc = 27;

    float speed = 0.0;
    float dir = 0.0;


    //getValues("~/pioneer2dx/camera/link/camera/image");
    //getValues("~/pioneer2dx/camera/link/camera/image);

  // gazebo::transport::SubscriberPtr valueSubscriber =
    //   node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", getValues);

      // FUZZY LOGIC

      Engine* engine = FllImporter().fromFile("../fuzzy_controller/LocalObstacleAvoidance_V2.fll");

      std::string status;
      if (not engine->isReady(&status))
      {
          throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
      }

      InputVariable* DirectionToObstacle = engine->getInputVariable("DirectionToObstacle");
      InputVariable* DistanceToObstacle = engine->getInputVariable("DistanceToObstacle");
      OutputVariable* Steer = engine->getOutputVariable("Steer");
      OutputVariable* Speed = engine->getOutputVariable("Speed");

      /*for (int i = 0; i <= 50; ++i)
      {
          scalar location = DirectionToObstacle->getMinimum() + i * (DirectionToObstacle->range() / 50);
          DirectionToObstacle->setValue(location);
          engine->process();
          FL_LOG("DirectionToObstacle.input = " << Op::str(location) << 
              " => " << "Steer.output = " << Op::str(Steer->getValue()));
      }*/


      //Initial speed
      speed += 0.1;


  std::cout << "Looping" << std::endl;
    // Loop
    while (true) {
      
      gazebo::common::Time::MSleep(10);

      mutex.lock();
      int key = cv::waitKey(1);
      mutex.unlock();

      DistanceToObstacle->setValue(fc.normalize(cb.getShortestRange(), "range"));
      DirectionToObstacle->setValue(fc.normalize(cb.getShortestAngle(), "angle"));

      engine->process();
    // FL_LOG("Steer.output = " << Op::str(Steer->getValue()));

      std::cout << "Range: " << fc.normalize(cb.getShortestRange(), "range") << "     ";
      std::cout << "Angle: " << fc.normalize(cb.getShortestAngle(), "angle") << "     ";
      std::cout << "Steer: " << Steer->getValue() << "     ";
      std::cout << "Speed: " << Speed->getValue() << std::endl;

      dir = (Steer->getValue()) * 5;
      speed = (Speed->getValue());

      if (DistanceToObstacle->getValue() == -1)
      {
          speed = -(Speed->getValue()) * 100;
          dir = (Steer->getValue()) * 100;
      }

      //std::cout << "angle: " << shortest_angle << "      " << "range: " << shortest_range << std::endl;   

      if (key == key_esc)
        break;

      if ((key == key_up) && (speed <= 1.2f))
        speed += 0.05;
      else if ((key == key_down) && (speed >= -1.2f))
        speed -= 0.05;
      else if ((key == key_right) && (dir <= 0.4f))
        dir += 0.05;
      else if ((key == key_left) && (dir >= -0.4f))
        dir -= 0.05;
      else {
        // slow down
        //      speed *= 0.1;
        //      dir *= 0.1;
      }

      // Generate a pose
      ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

      // Convert to a pose message
      gazebo::msgs::Pose msg;
      gazebo::msgs::Set(&msg, pose);
      movementPublisher->Publish(msg);
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
