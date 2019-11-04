#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "fl/Headers.h"
#include <math.h>

#include <iostream>

// Classes

#include "Fuzzy_control.h"
#include "Callback.h"
#include "MarbleDetection.h"

using namespace fl;

// Global variables

int main(int _argc, char **_argv) 
{
    static boost::mutex mutex;

    // Create objects
    Callback cb(cb.getCurPosition(), cb.getYaw());
    Fuzzy_control fc;
    MarbleDetection md;

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

    gazebo::transport::SubscriberPtr cameraSubscriber =
        node->Subscribe("~/pioneer2dx/camera/link/camera/image", &MarbleDetection::cameraCallback, &md);

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

    float range_min = 0.08;
    float range_max = 10;
    float lidarangle_min = 2.26889;
    float lidarangle_max = -2.2689;
    float angle_min = 0;
    float angle_max = 2*M_PI;


    //getValues("~/pioneer2dx/camera/link/camera/image");
    //getValues("~/pioneer2dx/camera/link/camera/image);

  // gazebo::transport::SubscriberPtr valueSubscriber =
    //   node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", getValues);

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


    // Loop
    while (true) 
    {      
      gazebo::common::Time::MSleep(10);

      mutex.lock();
      int key = cv::waitKey(1);
      mutex.unlock();

      // Setting input values
      DistanceToObstacle->setValue(fc.normalize(cb.getShortestRange(), range_min, range_max));
      DirectionToObstacle->setValue(fc.normalize(cb.getShortestAngle(), lidarangle_min, lidarangle_max));
      CornerType->setValue(cb.getCornerType());
      DirectionToGoal->setValue(fc.normalize(fc.angleToGoal(cb.getCurPosition(), cb.getYaw()), angle_min, angle_max));
      FreeLeftPassage->setValue(cb.getFreeLeftPassage());
      FreeRightPassage->setValue(cb.getFreeRightPassage());

      engine->process();

      // Printing values to the terminal
      /*std::cout << "Range: " << fc.normalize(cb.getShortestRange(), range_min, range_max) << "     ";
      std::cout << "Angle: " << fc.normalize(cb.getShortestAngle(), lidarangle_min, lidarangle_max) << "     ";
      std::cout << "DirectionToGoal: " << fc.normalize(fc.angleToGoal(cb.getCurPosition(), cb.getYaw()), angle_min, angle_max) << "     ";
      std::cout << "Steer: " << Steer->getValue() << "     ";
      std::cout << "Speed: " << Speed->getValue() << std::endl;*/

      //std::cout << "Left passage: " << cb.getFreeLeftPassage() << "     " << "Right passage: " << cb.getFreeRightPassage() << std::endl;

      // Setting output values
      dir = (Steer->getValue()) * 2;
      speed = (Speed->getValue());

      // If a corner is hit
      if (DistanceToObstacle->getValue() == -1)
      {
          speed = -(Speed->getValue()) * 100;
          //dir = (Steer->getValue()) * 100;
      }

      if (fc.distanceToGoal(cb.getCurPosition()) < 0.5)
      {
        speed = 0;
        dir = 0;
        std::cout << "You reached the goal!" << std::endl;
        break;
      }

      // Distance to goal
      //std::cout << fc.distanceToGoal(cb.getCurPosition()) << std::endl;

      // Angle to goal
      //std::cout << fc.angleToGoal(cb.getCurPosition(), cb.getYaw()) << "      "  <<  cb.getYaw()*180/M_PI << std:: endl;
      //std::cout << fc.normalize(fc.angleToGoal(cb.getCurPosition(), cb.getYaw()), angle_min, angle_max) << std::endl;

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
    //fc.drawPathSW(cb.getVector());
    fc.drawPathBW(cb.getVector());
}
