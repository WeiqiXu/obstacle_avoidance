/**************************************** *************************
* Filename: obstacle_avoidance_1.cpp
* Authors: Weiqi XU, wex064@eng.ucsd.edu
*		   Yiding QIU, yiqiu@eng.ucsd.edu
* Date: 02/06/2019
* HW #6: Non-verbal sensing & Multimodal controllers
*
*Description: This is the main control file for HW 6, 
*			it demonstrates a robot that finds the goal color patch 
*			and go towards it, avoiding the obstacles on its way.
			This program subscribes to the blob topic and point cloud topic.
			
*How to use:
 Usage:
*	roscore
*	roslaunch turtlebot_bringup minimal.launch
*	roslaunch astra_launch astra_pro.launch
*	
*	roslaunch cmvision colorgui image:=/camera/rgb/image_raw
*	<this is for color calibration, close when done>
*	roslaunch cmvision cmvision.launch image:=/camera/rgb/image_raw
*	<cnrl-c to kill process>
*	rosparam set /cmvision/color_file ~turtlebot_ws/src/cmvision/colors.txt
*	rosrun cmvision cmvision image:=/camera/rgb/image_raw
*	
*	rosrun obstacle_avoidance obstacle_avoidance_1
*******************************************************************/

#include <kobuki_msgs/BumperEvent.h> 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <stdio.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>

ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/* define global variables and initialization */
bool got_obstacle = false;
bool got_goal_blobs = false;
bool arrive_at_goal = false;
/* use the centroid as the initialization */
double goal_loc_x = 320; //640/2
double goal_loc_y = 210; //420/2
double obstacle_loc_x = 0;
double obstacle_loc_y = 0;

/************************************************************
 * Function Name: blobsCallBack
 * Parameters: const cmvision::Blobs
 * Returns: void
 * Description: This is the callback function of the /blobs topic;
 *				and gets the centroid of the color blobs (goal) if found.
 ***********************************************************/

void blobsCallBack (const cmvision::Blobs& blobsIn)
{
    double goal_area = 0; // the window size is 640*420 = 268800
    double goal_centroid_x = 0;
    double goal_centroid_y = 0;
    int goal_blobs_count = 0; //number of blobs detected

    if (blobsIn.blob_count > 0){
		for (int i = 0; i < blobsIn.blob_count; i++){
   			/* if green color blob is detected, add up the centroid coordinates and blob area. */
		    if (blobsIn.blobs[i].red == 0 && blobsIn.blobs[i].green == 255 && blobsIn.blobs[i].blue == 0){
                goal_blobs_count += 1;
		      	goal_centroid_x += blobsIn.blobs[i].x;
		      	goal_centroid_y += blobsIn.blobs[i].y;
                goal_area += blobsIn.blobs[i].area;
		    }		
		}
        /* if the goal area is larger than a threshold but not large enough, 
           it indicates that the goal is found but haven't arrived. */
        if (goal_area > 1000 && goal_area < 120000){
            //ROS_INFO("the goal area is: %f", goal_area);
            got_goal_blobs = true;
            ROS_INFO("We got a goal");
            /* get the average centroid of the blobs. */ 
            goal_loc_x = goal_centroid_x/goal_blobs_count;
            goal_loc_y = goal_centroid_y/goal_blobs_count;
        }
        /* if the goal area is larger than a threshold, then we have arrived at goal. */
        else if (goal_area >= 120000){
            ROS_INFO("arrived at goal");
            arrive_at_goal = true;    
    	}
	}
}

/************************************************************
 * Function Name: PointCloud_Callback
 * Parameters: const PointCloud::ConstPtr
 * Returns: void
 * Description: This is the callback function of the PointCloud
 * 				topic, flags when an obstacle is found. 
 ***********************************************************/
void PointCloud_Callback (const PointCloud::ConstPtr& cloud){

	double ZTHRESH = 0.9; // threshold under which a point is considered as obstacles
	int point_count = 0; // number of points within the threshold
	/* add-up coordinates of the obstacle points. */
	double pc_x = 0; 
	double pc_y = 0;
	got_obstacle = false;

    /* Iterate through all the points in the window to detect obstacle. */
  	for (int k = 0; k < 420; k++){
	    for (int i = 0; i < 640; i++){
			const pcl::PointXYZ & pt=cloud->points[640*(k)+(i)];
			if (pt.z < ZTHRESH){
				point_count += 1;
				pc_x += pt.x;
				pc_y += pt.y;
			}
	    }
  	}
  	obstacle_loc_x = pc_x/point_count;
	obstacle_loc_y = pc_y/point_count;
    ROS_INFO("point_count: %d", point_count);
	if (point_count > 0){ 
    	got_obstacle = true;
    	std::cout << "obstacle found" << std::endl;
    	//ROS_INFO("point_count is: %d", point_count);
  	}

}

/************************************************************
 * Main function
 * Description: Subscribes to the blob topic and PointCloud topic,
 *				publishes geometry message twist message. 
 * Function:	Go towards the goal and avoid obstacles on the way. 
 ***********************************************************/

int main (int argc, char** argv)
{
  // Initialize ROS
	ros::init (argc, argv, "blob");
	ros::NodeHandle nh;

	/* subscribe to /blobs topic */
	ros::Subscriber blobsSubscriber = nh.subscribe("/blobs", 1, blobsCallBack);
	/* subscribe to /PointCloud topic */
	ros::Subscriber PCSubscriber = nh.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);
	/* publish the geometry message twist message */
	ros::Publisher velocityPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

	ros::Rate loop_rate(1);
	geometry_msgs::Twist T;

	while (ros::ok()){

		T.linear.x = 0.0;T.linear.y = 0.0; T.linear.z = 0.0;
		T.angular.x = 0.0; T.angular.y = 0.0;T.angular.z = 0.0;

		/* state: obstacle found, avoid the obstacle */
		if (got_obstacle == true && arrive_at_goal == false){ 
		    ROS_INFO("obstacle_loc_x: %f, obstacle_loc_y: %f", obstacle_loc_x, obstacle_loc_y);
		    T.linear.x = 0.25;
		    
		    if (obstacle_loc_x < 0){
		      	ROS_INFO("obstacle at left, moving to right");  
		      	T.angular.z = -0.4; 
		    } 
		    else{
		        ROS_INFO("obstacle at right, moving to left");
		        T.angular.z = 0.4;
		    }
		    got_obstacle = false;
		    obstacle_loc_x = 0;   
		    obstacle_loc_y = 0;
		}
		/* state: goal found and no obstacle at sight: moving towards goal */
		else if (got_goal_blobs && arrive_at_goal == false){
		    ROS_INFO("the goal centroid is: goal_loc_x = %f, goal_loc_y = %f", goal_loc_x, goal_loc_y);

		    if (goal_loc_x < 390 && goal_loc_x > 250){
		        T.linear.x = 0.1;
		        T.angular.z = 0.0;
		        ROS_INFO("goal at center");
		    }
		    else{
		        ROS_INFO("turning towards the goal");
		        T.linear.x = 0.2;
		        /* change position to put goal at the center view */
		        if (goal_loc_x >= 390){
		            T.angular.z = -0.5;
		            ROS_INFO("goal at right, turning to right");
		        }
		        else if (goal_loc_x <= 250){
		            T.angular.z = 0.5;
		            ROS_INFO("goal at left, turning to left");
		        }
		    }   
		    got_goal_blobs = false;
		    goal_loc_x = 320;
		}
		/* state: reach the goal, stop */
		else if (arrive_at_goal){
			T.linear.x = 0.0;
			T.angular.z = 0.0;
			ROS_INFO("reach the goal!");
			break;
		} 
		/* state: no goal in sight, spin */
		else{ 
		    T.angular.z = -0.3; 
		    std::cout << "looking for goal" << std::endl;
		}

		// Spin
		ros::spinOnce();
		loop_rate.sleep();
		velocityPublisher.publish(T);

		}
}

