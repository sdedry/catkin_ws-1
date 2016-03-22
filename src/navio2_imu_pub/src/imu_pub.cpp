#include "MPU9250.h"
#include "LSM9DS1.h"
#include "Util.h"
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <sstream>

void init_imu_msg(sensor_msgs::Imu* imu_msg)
{
	//time stamp
	imu_msg->header.stamp = ros::Time::now();
	
	imu_msg->orientation.x = 0.0f;
	imu_msg->orientation.y = 0.0f;
	imu_msg->orientation.z = 0.0f;
	imu_msg->orientation.w = 0.0f;

	imu_msg->angular_velocity.x = 0.0f;
	imu_msg->angular_velocity.y = 0.0f;
	imu_msg->angular_velocity.z = 0.0f;

	imu_msg->linear_acceleration.x = 0.0f;
	imu_msg->linear_acceleration.y = 0.0f;
	imu_msg->linear_acceleration.z = 0.0f;

	//Create a cov matrix of [1 0 0; 0 1 0; 0 0 1]
	for(int i = 0; i < 9; i++)
	{
		imu_msg->orientation_covariance[i] = !(i%4);
		imu_msg->angular_velocity_covariance[i] = !(i%4);
		imu_msg->linear_acceleration_covariance[i] = !(i%4);
	}
}

void init_mf_msg(sensor_msgs::MagneticField* mf_msg)
{
	//time stamp
	mf_msg->header.stamp = ros::Time::now();
	
	mf_msg->magnetic_field.x = 0.0f;
	mf_msg->magnetic_field.y = 0.0f;
	mf_msg->magnetic_field.z = 0.0f;

	//Create a cov matrix of [1 0 0; 0 1 0; 0 0 1]
	for(int i = 0; i < 9; i++)
	{
		mf_msg->magnetic_field_covariance[i] = !(i%4);
	}
}

void update_imu_msg(sensor_msgs::Imu* imu_msg, InertialSensor* sensor)
{
	//time stamp
	imu_msg->header.stamp = ros::Time::now();
	
	float ax, ay, az, gx, gy, gz;

	sensor->update();

	sensor->read_accelerometer(&ax, &ay, &az);
        sensor->read_gyroscope(&gx, &gy, &gz);
        //sensor->read_magnetometer(&mx, &my, &mz);

	//imu_msg->orientation.x = ax;
	//imu_msg->orientation.y = ay;
	//imu_msg->orientation.z = az;

	imu_msg->angular_velocity.x = gx;
	imu_msg->angular_velocity.y = gy;
	imu_msg->angular_velocity.z = gz;

	imu_msg->linear_acceleration.x = ax;
	imu_msg->linear_acceleration.y = ay;
	imu_msg->linear_acceleration.z = az;

	ROS_INFO("Accelerometer : X = %+7.3f, Y = %+7.3f, Z = %+7.3f", ax, ay, az);
	ROS_INFO("Gyroscope : X = %+7.3f, Y = %+7.3f, Z = %+7.3f", gx, gy, gz);
}

void update_mf_msg(sensor_msgs::MagneticField* mf_msg, InertialSensor* sensor)
{
	//time stamp
	mf_msg->header.stamp = ros::Time::now();
	
	float mx, my, mz;

	sensor->update();

        sensor->read_magnetometer(&mx, &my, &mz);

	mf_msg->magnetic_field.x = mx;
	mf_msg->magnetic_field.y = my;
	mf_msg->magnetic_field.z = mz;

	ROS_INFO("Magnetic Field : X = %+7.3f, Y = %+7.3f, Z = %+7.3f", mx, my, mz);
}

int main(int argc, char **argv)
{
 	/***********************/
	/* Initialize The Node */
	/***********************/
	ros::init(argc, argv, "imu_lsm9ds1_handler");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_readings", 1000);
	ros::Publisher mf_pub = n.advertise<sensor_msgs::MagneticField>("mag_readings", 1000);

	//running rate = 2 Hz
	ros::Rate loop_rate(2);

	/*************************/
	/* Initialize the Sensor */
	/*************************/

	printf("Selected: LSM9DS1\n");
	InertialSensor* sensor = new LSM9DS1();

	/***************/
	/* Test Sensor */
	/***************/
	if (!sensor->probe()) 
	{
		printf("Sensor not enabled\n");
		return EXIT_FAILURE;
	}

	sensor->initialize();

	while (ros::ok())
	{

		sensor_msgs::Imu imu_msg;
		sensor_msgs::MagneticField mf_msg;

		init_imu_msg(&imu_msg);
		init_mf_msg(&mf_msg);
		
		update_imu_msg(&imu_msg, sensor);
		update_mf_msg(&mf_msg, sensor);

		imu_pub.publish(imu_msg);
		mf_pub.publish(mf_msg);

		ros::spinOnce();

		loop_rate.sleep();

	}


  	return 0;
}

