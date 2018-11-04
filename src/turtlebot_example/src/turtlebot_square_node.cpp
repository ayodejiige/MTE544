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
#include <mutex>

//Callback function for the Position topic 
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
	//This function is called when a new position message is received

//	double X = msg->pose.pose.position.x; // Robot X psotition
//	double Y = msg->pose.pose.position.y; // Robot Y psotition
//	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

//}

std::mutex g_mtx;

enum states:u_int16_t
{
	TURN_0,
	Y_NEG,
	TURN_90,
	X_POS,
	TURN_180,
	Y_POS,
	TURN_270,
	X_NEG,
};

double current_x = 0;
double current_y = 0;
double current_yaw = 0;
volatile bool started = false;

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received
	started = true;
	// g_mtx.lock();
	current_x = msg->pose.pose.position.x; // Robot X psotition
	current_y = msg->pose.pose.position.y; // Robot Y psotition 
	current_yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
	// new_data = true;
	// g_mtx.unlock();
	ROS_INFO("x : %.4f | y : %.4f | yaw : %.4f", current_x, current_y, current_yaw);
}


float set_yaw(float target, float current)
{
	static float kp = 0.7;
	float error = target - current;
	float control = kp * error;

	return control;
}

float set_pos(float target, float current)
{
	static float kp = 0.5;
	float error = target - current;
	float control = kp * error;

	return control;
}

int main(int argc, char **argv)
{
    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

	static const double width = 1;
	double target_pos = 0;
	u_int16_t current_state = TURN_0;
	int loopCount = 0;
	
    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
		
		double linear_ctrl = 0;
		double angular_ctrl = 0;
		// g_mtx.lock();
		if(!started) continue;
		// if(!new_data)
		// {
		// 	g_mtx.unlock();
		// 	continue;
		// }

		switch(current_state)
		{
			case TURN_0:
				// ROS_INFO("here");
				linear_ctrl = 0;
				angular_ctrl = set_yaw(0, current_yaw);
				if(fabs(current_yaw) < 0.1)
				{
					current_state = Y_NEG;
					target_pos = current_y - width;
					ROS_INFO("Switching from [%d] to [%d]", TURN_0, current_state);
				}
				break;
			case Y_NEG:
				linear_ctrl = 0.2;
				angular_ctrl = set_yaw(0, current_yaw);
				loopCount++;
				if (loopCount % 100 == 0)
				{
					ROS_INFO("starting turning\n");
					current_state = TURN_90;
				}
				// linear_ctrl = fabs(set_pos(target_pos, current_y));
				// angular_ctrl = set_yaw(0, current_yaw);
				// if(fabs(target_pos - current_y) < 0.1)
				// {
				// 	current_state = TURN_90;
				// 	ROS_INFO("Switching from [%d] to [%d]", Y_NEG, current_state);
				// }
				break;
			case TURN_90:
				linear_ctrl = 0;
				angular_ctrl = set_yaw(1.5708, current_yaw);
				if(fabs(current_yaw - 1.5708) < 0.1)
				{
					current_state = X_POS;
					target_pos = current_x + width;
					ROS_INFO("Switching from [%d] to [%d]", TURN_90, current_state);
				}
				break;
			case X_POS:
				linear_ctrl = 0.2;
				angular_ctrl = set_yaw(1.5708, current_yaw);
				loopCount++;
				if (loopCount % 100 == 0)
				{
					ROS_INFO("starting turning\n");
					current_state = TURN_180;
				}
				// linear_ctrl = fabs(set_pos(target_pos, current_x));
				// angular_ctrl = set_yaw(1.5708, current_yaw);
				// if(fabs(target_pos - current_x) < 0.1)
				// {
				// 	current_state = TURN_180;
				// 	ROS_INFO("Switching from [%d] to [%d]", X_POS, current_state);
				// }
				break;
			case TURN_180:
				linear_ctrl = 0;
				angular_ctrl = set_yaw(3.14159, fabs(current_yaw));
				// Correct for deadzone
				if(current_yaw < 0)
				{
					angular_ctrl *= -1;
				}
				if(fabs(fabs(current_yaw) - 3.14159) < 0.1)
				{
					current_state = Y_POS;
					target_pos = current_y + width;
					ROS_INFO("Switching from [%d] to [%d]", TURN_180, current_state);
				}
				break;
			case Y_POS:
				linear_ctrl = 0.2;
				angular_ctrl = set_yaw(3.14159, fabs(current_yaw));
				loopCount++;
				if (loopCount % 100 == 0)
				{
					ROS_INFO("starting turning\n");
					current_state = TURN_270;
				}
				// linear_ctrl = set_pos(target_pos, current_y);
				// angular_ctrl = set_yaw(3.14159, fabs(current_yaw));
				// if(fabs(target_pos - current_y) < 0.1)
				// {
				// 	current_state = TURN_270;
				// 	ROS_INFO("Switching from [%d] to [%d]", Y_POS, current_state);
				// }
				break;
			case TURN_270:
				linear_ctrl = 0;
				angular_ctrl = set_yaw(-1.5708, current_yaw);
				if(fabs(current_yaw + 1.5708) < 0.1)
				{
					current_state = X_NEG;
					target_pos = current_x - width;
					ROS_INFO("Switching from [%d] to [%d]", TURN_270, current_state);
				}
				break;
			case X_NEG:
				linear_ctrl = 0.2;
				angular_ctrl = set_yaw(-1.5708, current_yaw);
				loopCount++;
				if (loopCount % 100 == 0)
				{
					ROS_INFO("starting turning\n");
					current_state = TURN_0;
				}
				// linear_ctrl = fabs(set_pos(target_pos, current_x));
				// angular_ctrl =  set_yaw(-1.5708, current_yaw);
				// if(fabs(target_pos - current_x) < 0.1)
				// {
				// 	current_state = TURN_0;
				// 	ROS_INFO("Switching from [%d] to [%d]", X_NEG, current_state);
				// }
				break;
			default:
				linear_ctrl = 0.0;
				angular_ctrl = 0.0;
				break;
		}
		// new_data = false;
		// g_mtx.unlock();
    	//Main loop code goes here:
		vel.linear.x = linear_ctrl;
		vel.angular.z = angular_ctrl;
		velocity_publisher.publish(vel); // Publish the command velocity

		// ROS_INFO("angular: %.5f, linear: %.5f", angular_ctrl, linear_ctrl);
	}
    return 0;
}
