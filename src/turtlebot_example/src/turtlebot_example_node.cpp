//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cmath>

//Callback function for the Position topic 
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
	//This function is called when a new position message is received

//	double X = msg->pose.pose.position.x; // Robot X psotition
//	double Y = msg->pose.pose.position.y; // Robot Y psotition
//	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

//}

// Before starting to turn, capture yaw value
double initial_yaw = 0;

// Store current yaw value here in every callback
double current_yaw = 0;

// Used to count how many interations of the loop have passed
int loopCount = 0;

// If true, the robot is turning 90 degrees. Else, moving linearly
bool isTurning = false;

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

	double X = msg->pose.pose.position.x; // Robot X psotition
	double Y = msg->pose.pose.position.y; // Robot Y psotition 
 	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
	
	current_yaw = Yaw;
}



int main(int argc, char **argv)
{
    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
    
    	//Main loop code goes here:

	// Linear motion
	if (!isTurning)
	{
		vel.linear.x = 0.2;
		vel.angular.z = 0;
		loopCount++;

		// Stop linear motion, start turning at this point
		// Change 100 value to change side length
		if (loopCount % 100 == 0)
		{
			initial_yaw = current_yaw;
			isTurning = true;
		}
	}
	else // Turning motion
	{
		vel.linear.x = 0;
		vel.angular.z = 0.2;

		// The yaw value goes from -PI to +PI, need to account for this
		// Turn 90 degrees (PI/2 rads) every time to form square
		if (initial_yaw > current_yaw)
		{
			if ((2 * M_PI + current_yaw - initial_yaw) > (M_PI/2.0))
			{
				isTurning = false;
			}
		}
		else
		{
			if (std::abs(initial_yaw - current_yaw) > (M_PI/2.0))
			{
				isTurning = false;
			}
		}
	}

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
