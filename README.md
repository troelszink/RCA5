# RCA5-Project for 5th semester on Civ. Robot Engineering, SDU

## READ THIS BEFORE TESTING OUR SOLUTIONS

There are 5 major topics divided into 6 mains. They are as follows:

- **mainAI-FC.cpp**         - Main for Fuzzy Control
-- Set the goal for the Fuzzy Controller in mainAI-FC.h
- **mainAI-QL.cpp**         - Main for Q-Learning
- **mainVIS.cpp**           - Main for image analysis with Computer Vision
- **mainROB-WF.cpp**        - Main for Robotics, Wavefront Planner
- **mainROB-BF.cpp**        - Main for Robotics, Brushfire Algorithm and the GVD
- **mainROB-Loc.cpp**       - Main for Robotics, Localization

These mains are located in the folder: /robot_control/src/mains/
- The one called "main.cpp" in the /robot_control/src/ folder is the one called: **"mainAI-FC.cpp"**.

However, in /robot_control/src there are one main, and the rest are in "mains".
This is the main-file that will be executed when building the project.
Therefore, the main you will like to execute should be moved to: /robot_control/src and renamed to "main.cpp". Move the old main-file into the "mains"-folder and renamed it according to the name definitions above.

Alternatively, you can change the name of the main-file you will like to execute in the build-files.
