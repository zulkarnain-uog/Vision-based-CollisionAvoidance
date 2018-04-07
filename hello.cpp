//Used to test reaction.cpp without camera input
//Arbitrary values sent 

#include <algorithm>
#include <functional>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros/mavros.h"
#include "mavros/px4_custom_mode.h"
#include "mavros_msgs/OverrideRCIn.h"
#include <ros/console.h>
#include <mavros_msgs/RCOut.h>
#include <sensor_msgs/JointState.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include "rosconsole/macros_generated.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/PositionTarget.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;



int main( int argc, char **argv )
{
	//Initialize node and node name
	ros::init(argc, argv, "detection"); //Node called 'detection'
	ros::NodeHandle nh1;
	//advertise as detection
	ros::Publisher camera_det = nh1.advertise<std_msgs::Float64MultiArray>("/detection", 2); //Pub topic called 'Obsdistance'
	ros::Rate loop_rate(2);
	float R;
	float L;
	float C;

	int i;
	cout << "select scenario (1 no obs, 2 Right obs, 3 Left obs, 4 Center obs): \n";
	cin >> i;
	switch(i)
	{
		case 1: 	for(int count = 0; count < 100; count++)
					{
						float R = 4;
						float L = 3;
						float C = 2;
						while(ros::ok())
						{
							//Set up publishing topic
							std_msgs::Float64MultiArray Obsdistance; //Declare Obsdistance
							Obsdistance.data.resize(2); //Resize the array to assign to existent values
							Obsdistance.data[0] = R; //R to first element in the array
							Obsdistance.data[1] = L; //L to first element in the array
							Obsdistance.data[2] = C; //C to first element in the array

							ROS_INFO("Right: %f", R);
							ROS_INFO("Left: %f", L);
							ROS_INFO("Center: %f", C);
							camera_det.publish(Obsdistance);
							ros::spinOnce();
							loop_rate.sleep();

						}

					}
					break;
		case 2: 	for(int count = 0; count < 100; count++)
							{
								float R = 0.9;
								float L = 3;
								float C = 2;
								while(ros::ok())
								{
									//Set up publishing topic
									std_msgs::Float64MultiArray Obsdistance; //Declare Obsdistance
									Obsdistance.data.resize(2); //Resize the array to assign to existent values
									Obsdistance.data[0] = R; //R to first element in the array
									Obsdistance.data[1] = L; //L to first element in the array
									Obsdistance.data[2] = C; //C to first element in the array

									ROS_INFO("Right: %f", R);
									ROS_INFO("Left: %f", L);
									ROS_INFO("Center: %f", C);
									camera_det.publish(Obsdistance);
									ros::spinOnce();
									loop_rate.sleep();

								}
							}
							break;
		case 3: 	for(int count = 0; count < 100; count++)
							{
								float R = 4;
								float L = 0.9;
								float C = 2;
								while(ros::ok())
								{
									//Set up publishing topic
									std_msgs::Float64MultiArray Obsdistance; //Declare Obsdistance
									Obsdistance.data.resize(2); //Resize the array to assign to existent values
									Obsdistance.data[0] = R; //R to first element in the array
									Obsdistance.data[1] = L; //L to first element in the array
									Obsdistance.data[2] = C; //C to first element in the array

									ROS_INFO("Right: %f", R);
									ROS_INFO("Left: %f", L);
									ROS_INFO("Center: %f", C);
									camera_det.publish(Obsdistance);
									ros::spinOnce();
									loop_rate.sleep();

								}
							}
							break;
		case 4: 	for(int count = 0; count < 100; count++)
							{
								float R = 4;
								float L = 3;
								float C = 0.9;
								while(ros::ok())
								{
									//Set up publishing topic
									std_msgs::Float64MultiArray Obsdistance; //Declare Obsdistance
									Obsdistance.data.resize(2); //Resize the array to assign to existent values
									Obsdistance.data[0] = R; //R to first element in the array
									Obsdistance.data[1] = L; //L to first element in the array
									Obsdistance.data[2] = C; //C to first element in the array

									ROS_INFO("Right: %f", R);
									ROS_INFO("Left: %f", L);
									ROS_INFO("Center: %f", C);
									camera_det.publish(Obsdistance);
									ros::spinOnce();
									loop_rate.sleep();

								}
							}
							break;
	}




}
