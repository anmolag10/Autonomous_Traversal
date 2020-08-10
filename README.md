
# Autonomous_Traversal
 Gazebo simulation of a 4 wheeled skid steer bot implementing autonomous traversal without  as well as traversal with obstacle avoidance using a GPS, an IMU, Kinect.

## Installation
   1. Install:
      ```bash
         sudo apt-get install ros-melodic-depthimage-to-laserscan
      ```
   2. Clone repo:
      ```bash
         git clone https://github.com/anmolag10/Autonomous_Traversal/
      ```
 
   4. Then run:
      ```bash
         roslaunch mybot_gazebo mybot_world.launch
         
         cd src/mybot_description
         chmod +x plaintraversal.py
         rosrun mybot_description plaintraversal.py
         
         cd src/mybot_description
         chmod +x obstacleavoidance.py
         rosrun mybot_description obstacleavoidance.py
      ```
   5.GPS cordinates of the goal can be edited in the script by extracting them from rostopic /fix
