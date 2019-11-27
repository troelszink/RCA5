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
#include "Localization.h"

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
    Localization loc;
    srand(time(NULL));

    md.addObject(cb);

    cb.setSensorIncrement(10); // Go from 200 sensors to only 20. Faster computation.

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

    /*gazebo::transport::SubscriberPtr cameraSubscriber =
        node->Subscribe("~/pioneer2dx/camera/link/camera/image", &MarbleDetection::cameraCallback, &md);*/

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

      // Localization
      std::vector<particle> particleVector = loc.generateParticles(500);
      // Making a route for the robot to go
      std::vector<cv::Point2f> pathVector;
      for (int i = 0; i < 20; i++)
      {
          pathVector.push_back(cv::Point2f(i, 0));
      }
      for (int i = 1; i < 20; i++)
      {
          pathVector.push_back(cv::Point2f(19, i));
      }
      for (int i = 1; i < 20; i++)
      {
          pathVector.push_back(cv::Point2f(19 + i, 19));
      } 
      int goalCounter = 0;

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

      // Setting output values
      dir = (Steer->getValue()) * 2;
      speed = (Speed->getValue());

      // Checking the end goal for path planning
      if (abs(cb.getCurPosition().x - pathVector[goalCounter].x) < 0.2 && abs(cb.getCurPosition().y - pathVector[goalCounter].y) < 0.2)
      {
          goalCounter++;
          speed = 0;
          dir = 0;

          // Generate a pose
          ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

          // Convert to a pose message
          gazebo::msgs::Pose msg;
          gazebo::msgs::Set(&msg, pose);
          movementPublisher->Publish(msg);

          float scaling = 1.41735;

          float deltaX = (pathVector[goalCounter].x - pathVector[goalCounter - 1].x) * scaling;
          float deltaY = (pathVector[goalCounter].y - pathVector[goalCounter - 1].y) * scaling;
          float length = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

          for (int i = 0; i < particleVector.size(); i++)
          {
                // Muligivs ændre tilbage til hvad det var før
                if (pathVector[goalCounter].y > pathVector[goalCounter - 1].y)
                {
                    particleVector[i].coord.x += deltaX;
                    particleVector[i].coord.y -= deltaY;
                }
                else if (pathVector[goalCounter].y <= pathVector[goalCounter - 1].y)
                {
                    particleVector[i].coord.x += deltaX;
                    particleVector[i].coord.y += deltaY;
                }

                /*if (particleVector[i].yaw >= 0 && particleVector[i].yaw < 0.5*M_PI) // 1st quadrant
                {
                    particleVector[i].coord.x += cos(particleVector[i].yaw) * length;
                    particleVector[i].coord.y += -sin(particleVector[i].yaw) * length;
                } 
                else if (particleVector[i].yaw >= 0.5*M_PI && particleVector[i].yaw < M_PI) // 2nd quadrant
                {
                    particleVector[i].coord.x += -cos(M_PI - particleVector[i].yaw) * length;
                    particleVector[i].coord.y += -sin(M_PI - particleVector[i].yaw) * length;
                }
                else if (particleVector[i].yaw >= -M_PI && particleVector[i].yaw < -0.5*M_PI) // 3rd quadrant
                {
                    particleVector[i].coord.x += -cos(M_PI + particleVector[i].yaw) * length;
                    particleVector[i].coord.y += sin(M_PI + particleVector[i].yaw) * length;
                }
                else if (particleVector[i].yaw >= -0.5*M_PI && particleVector[i].yaw < 0) // 4th quadrant
                {
                    particleVector[i].coord.x += cos(particleVector[i].yaw) * length;
                    particleVector[i].coord.y += -sin(particleVector[i].yaw) * length; // Negative "-" as we need y to be positive, but it becomes negative, because of negative yaw
                }*/
          }

          particleVector = loc.updateWeigths(particleVector, cb.getRangeVector(), cb.getCurPosition());
          fc.setGoal(pathVector[goalCounter]);
          std::cout << "You reached the goal!" << std::endl;
      }

      std::cout << "Current position: " << cb.getCurPosition().x << "," << cb.getCurPosition().y << std::endl;

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
   // fc.drawPathBW(cb.getVector());
}
