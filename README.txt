
/***********************************************/

Authors: Weiqi XU, wex064@eng.ucsd.edu
	 Yiding Qiu, yiqiu@eng.ucsd.edu
Date: 02/06/2019
Description: This is a ROS package for turtlebot which realizes object tracking while 		     avoiding obstacles along the way. CMvision and PointCloud library 
	     are used to complete this task.

/***********************************************/

The executable file is obstacle_avoidance_1.cpp under src folder.
To get the program running, simply do:

> roscore
> roslaunch turtlebot_bringup minimal.launch
> roslaunch astra_launch astra_pro.launch
	
> roslaunch cmvision colorgui image:=/camera/rgb/image_raw
  <this is for color calibration, close when done>
> roslaunch cmvision cmvision.launch image:=/camera/rgb/image_raw
  <cnrl-c to kill process>
> rosparam set /cmvision/color_file ~turtlebot_ws/src/cmvision/colors.txt
> rosrun cmvision cmvision image:=/camera/rgb/image_raw 

> rosrun obstacle_avoidance obstacle_avoidance_1

Note that the source code is write up to only take green color patch as the goal. To change the goal color, re-calibration of colors is needed. The RGB values and the corresponding AB values need to be stored in colors.txt file.
