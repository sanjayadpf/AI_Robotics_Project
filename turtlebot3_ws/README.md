**Clone the project**

git clone https://github.com/sanjayadpf/AI_Robotics_Project


**Go inside the src directory of turtlebot3_ws and run the following command**

git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

**After that go back to turtlebot3_ws and run the following**

catkin_make

**Open seperate terminals for each functionality**

# terminal 01 Gazebo


source devel/setup.bash

roslaunch ros_world turtlebot3_world.launch


# terminal 02 Run Rviz


source devel/setup.bash

roslaunch path_planning turtlebot3_ros_world.launch

--For the actual robot (Don't run Gazebo following launch file intiate the master node)

--You don't need to run this for simulations

roslaunch path_planning turtlebot3_ros_real_world.launch

# terminal 03 real time SLAM


source devel/setup.bash

roslaunch turtlebot3_slam turtlebot3_gmapping.launch

# terminal04 Path Planning Algorithms (optional)

rosrun server.py (incase the launch file unable to load the script)


