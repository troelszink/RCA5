#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "fl/Headers.h"
#include <math.h>

#include <iostream>
#include <stdlib.h>

// Classes

#include "Fuzzy_control.h"
#include "Callback.h"
#include "MarbleDetection.h"
#include "PathPlanning.h"

using namespace fl;

// Global variables

int main(int _argc, char **_argv) 
{
    static boost::mutex mutex;

    // Create objects
    Callback cb(cb.getCurPosition(), cb.getYaw());
    Fuzzy_control fc;
    MarbleDetection md;
    PathPlanning pp;

    md.addObject(cb);

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

    const int key_esc = 27;

    float speed = 0.0;
    float dir = 0.0;

      // FUZZY LOGIC
      Engine* engine = FllImporter().fromFile("../fuzzy_controller/LocalObstacleAvoidance_V3.fll");

      std::string status;
      
      if (not engine->isReady(&status))
      {
          throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
      }

      InputVariable* DirectionToObstacle = engine->getInputVariable("DirectionToObstacle");
      InputVariable* DistanceToObstacle = engine->getInputVariable("DistanceToObstacle");
      InputVariable* CornerType = engine->getInputVariable("CornerType");
      InputVariable* DirectionToGoal = engine->getInputVariable("DirectionToGoal");
      InputVariable* FreeLeftPassage = engine->getInputVariable("FreeLeftPassage");
      InputVariable* FreeRightPassage = engine->getInputVariable("FreeRightPassage");
      OutputVariable* Steer = engine->getOutputVariable("Steer");
      OutputVariable* Speed = engine->getOutputVariable("Speed");

    // Path planning
    std::vector<cellValueWF> wavefrontVec = pp.wavefront();

    // Loop
    while (true) 
    {      
      gazebo::common::Time::MSleep(10);

      mutex.lock();
      int key = cv::waitKey(1);
      mutex.unlock();

      // Setting input values
      DirectionToGoal->setValue(fc.normalize(fc.angleToGoal(cb.getCurPosition(), cb.getYaw()), angle_min, angle_max));

      engine->process();

      // Setting output values
      dir = (Steer->getValue()) * 2;
      speed = (Speed->getValue()) / 5;


      // Checking the end goal for path planning
      if (abs(cb.getCurPosition().x - pp.getGoal().x) < 0.5 && abs(cb.getCurPosition().y - pp.getGoal().y) < 0.5)
      {
            speed = 0;
            dir = 0;
            // Generate a pose
            ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));
            // Convert to a pose message
            gazebo::msgs::Pose msg;
            gazebo::msgs::Set(&msg, pose);
            movementPublisher->Publish(msg);

            std::cout << "You reached the goal!" << std::endl;
            break;
      }

      if (fc.distanceToGoal(cb.getCurPosition()) < 0.3)
      {
          std::cout << "WayPoint reached" << std::endl;
          cv::Point2f wayPoint = pp.robotControl(wavefrontVec, cb.getCurPosition());
          fc.setGoal(wayPoint);
      }

      if (key == key_esc)
        break;

      // Generate a pose
      ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

      // Convert to a pose message
      gazebo::msgs::Pose msg;
      gazebo::msgs::Set(&msg, pose);
      movementPublisher->Publish(msg);
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();

    // Draw path for the robot
    fc.drawPathSW(cb.getVector());
}
