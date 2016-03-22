#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


int main(int argc, char **argv)
{
 	/***********************/
	/* Initialize The Node */
	/***********************/
	ros::init(argc, argv, "imu_readings");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("Acceleration", 1000);

	//running rate = 2 Hz
	ros::Rate loop_rate(2);

	while (ros::ok())
	{
		ROS_INFO("Good Morning ");

		ros::spinOnce();

		loop_rate.sleep();

	}


  	return 0;
}

