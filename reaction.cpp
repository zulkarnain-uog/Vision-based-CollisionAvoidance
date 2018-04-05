	/*Description of collision avoidance algorithm and reactive motion
	 *
	 * UAV will only potentially move in other modes when obstacle is detected else UAV stayed moving straight.
	 * Detection boundary set at 1m.
	 * If obstacle is at center, it will move Right. Moving up is too risky due to ceiling restrictions.
	 * Since FoV of cam is about 60 deg, need to elevate 0.5m (sin 30 = h/1) to clear obstacle to clear avoidance algorithm.
	 * Potential problem is also path planning after avoiding is more unknown than going around.
	 *
	 * Therefore:
	 * No obstacle : Move straight
	 * Obstacle detected : Check detection values to decide
	 * How? By calculating nearest pixel and determining which frame that is.
	 * Assume : NO drift, depth is accurate, fairly uniform input from detection side, non-dynamic obstacles?
	 * Issues: How to get back to track after avoiding object? (SLAM feature provided from camera?)
	 *
	 * Who is needed to make this work?
	 * Detection side: camera depth values from each frame. (in terms of L, C, R in meters)
	 * Pixhawk: To control Power Distribution Board (PDB) to control UAV motion. (Use mavros/OverrideRCIn or PWM)
	 * 			Gyro and Magnetometer: To calculate attitude of UAV and orientation respectively.
	 * 			Accelerometer: To measure acceleration.
	 * 			Velocity and Pose from odometry perhaps?
	 * Reaction side: Pixhawk input of current state (pose, vel, acc, orientation) and distance of obstacle from UAV.
	 * 				  Figure out command type to manipulate motor controls to suit control needs.
	 * 				  Implement some form of controller (PID?) and filter (EKF?) to input from pixhawk for cleaner response.
	 *
	 * How to react to distance given: Use command manipulation either into OverrideRCIn or PWM to control motor.
	 * 								   Use imaginary spherical boundary, hard coded to be 1 meter, to define threshold of avoidance.
	 *
	 * How to react when no obstacle: Follow flight plan pre-programmed before flight.
	 *
	 * How to determine scale of motion/velocity (linear and angular): Initial guess then Experimentation to refine constants
	 *
	 * What is needed to program?
	 *
	 * Node: Detection (depth Published from here) No Subcribers
	 * 		 Reaction
	 * 		 IMU       (Mavros handles all publishing and subscribing remotely)
	 *
	 * Subscribers: distance, IMU to check current params (pose, vel, acc, orientation, RCIn)
	 * Publishers: (Publish through mavros control command/input to motor via OverrideRCIn or PWM to IMU?)
	 * Data Types: Create one for publishing from detection.cpp to reaction.cpp using struct.
	 *
	 *
	 *
	 * ==================Final suggested method to output value from detection.cpp to reaction.cpp================================================
	 *
	 * Create data type using struct 'Obsdistance' containing floats (L, C, R) with 'frame_res' object.
	 * ie. frame_res.Left;
	 * 	   frame_res.Center;
	 * 	   frame_res.Right;
	 *
	 *     Publishing as : distance.publish(3_frame_res);
	 *
	 * Values of interest from detection.cpp: R, L, C.
	 * They will be given values from the R, L, C of detection results.
	 * The above will be published as a topic under "detection" from the detection node.
	 *
	 *
	 *
	 * ==================================Final suggested algorithm/flow of Obstacle Avoidance==================================
	 *
	 * Will only move straight if no obstacle is detected.
	 * Method of avoidance will be boundary-based on detection values retrieved from camera.
	 * Motion sequence will be refreshed periodically based on input from detection result.
	 * --Fill in method to bring back to original flight path -- (maybe pre-flight planning from MissionPlanner?)
	 * Algorithm loop: Move --> Detect --> Avoid --> Move back to path
	 */

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

double prev_Time = ros::Time::now().toSec();

//-----------------------------------------//
//---------------Classes-------------------//
//-----------------------------------------//
void move(bool detect_flag, bool shortestL,	bool shortestR,	bool shortestC,	double dshortest);
//void PID(double Error, double new_Error, double P, double I, double D);
//void shortestDist(double shortestL, double shortestR, double shortestC, double left, double center, double right, double dshortest);
//void cartesian_to_spherical(double x, double y, double z);
//void PID_RCIn(double OverrideRCIn_msg, double OverrideRCIn, double PID);
//void react(double dshortest, double sensor_msgs);
//void Tuning (double P, double I, double D, double Kp, double Ki, double Kd);

//-----------------------------------------//
//-------Subscriber and Publisher ---------//
//-----------------------------------------//
class SubscribeAndPublishIMU
{
public:
  SubscribeAndPublishIMU()
  {

	  current_IMU = nh.subscribe<sensor_msgs::Imu>("/sensor_msgs/Imu", 50, imu_cb);//Get IMU status and values

	  update_IMU = nh.advertise<sensor_msgs::Imu>("/sensor_msgs/Imu", 50);

  }

  void imu_cb (const sensor_msgs::ImuConstPtr& rcoutmsg)
  {
  	sensor_msgs::Imu IMU;

  	ROS_INFO("Angular velocity: ", IMU.angular_velocity);
  	ROS_INFO("Angular velocity covariance: ", IMU.angular_velocity_covariance);
  	ROS_INFO("linear acceleration: ", IMU.linear_acceleration);
  	ROS_INFO("linear acceleration covariance: ", IMU.linear_acceleration_covariance);

  	update_IMU.publish(IMU);
  }

private:
  ros::NodeHandle nh;
  ros::Publisher update_IMU;
  ros::Subscriber current_IMU;

};

class SubscribeAndPublishState
{
public:
  SubscribeAndPublishState()
  {

	  current_state = nh.subscribe<mavros_msgs::State>("/mavros_msgs/state", 60, state_cb);	//Get current UAV status

	  update_state = nh.advertise<mavros_msgs::State>("/mavros_msgs/state", 60);

  }

  void state_cb (const mavros_msgs::StateConstPtr& statemsg)
  {
  	mavros_msgs::State current_state;

  	// wait for FCU connection
      while(ros::ok() && !current_state.connected){
      	ros::spinOnce();
      	ros::Duration().sleep();
      }
      while(ros::ok() && current_state.connected)
      {
          mavros_msgs::CommandBool arm_cmd;
          arm_cmd.request.value = 1;
          ROS_INFO("Arming status: %s", arm_cmd.Response);
      }


  //    mavros_msgs::SetMode offb_set_mode;
  //    offb_set_mode.request.custom_mode = "OFFBOARD";
  //
  //    ros::Time last_request = ros::Time::now();
  //
  //    while(ros::ok()){
  //        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
  //        {
  //            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
  //            {
  //                ROS_INFO("Offboard enabled");
  //            }
  //            last_request = ros::Time::now();
  //        } else {
  //            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
  //            {
  //                if( arming_client.call(arm_cmd) && arm_cmd.)
  //                {
  //                    ROS_INFO("Vehicle armed");
  //                }
  //                last_request = ros::Time::now();
  //            }
  //        }
      	update_state.publish(current_state);
        ros::spinOnce();
        ros::Duration().sleep();
  }

private:
  ros::NodeHandle nh;
  ros::Publisher update_state;
  ros::Subscriber current_state;

};

class SubscribeAndPublishInitiate
{
public:
  SubscribeAndPublishInitiate()
  {

	  local_pose = nh.subscribe<mavros_msgs::PositionTarget>("/mavros_msgs/PositionTarget", 60, PoseTarg_cb);				//Get Current pose
	  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros_msgs/setpoint_position/local", 60);	//Send goal pose to UAV
	  goal = nh.advertise<mavros_msgs::State>("/mavros_msgs/state", 60);

  }

void PoseTarg_cb (const mavros_msgs::PositionTargetConstPtr& rcoutmsg)
{
	mavros_msgs::State current_state;
	if(ros::ok() && current_state.connected)
    {

    //move quad to starting position

    	geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;
        ros::Duration(5).sleep(); //sleep for 5 seconds

        local_pos_pub.publish(pose);
    }

}

private:
  ros::NodeHandle nh;
  ros::Publisher goal;
  ros::Subscriber local_pose;
  ros::Publisher local_pos_pub;

};
//-----------------------------------------//
//---------Subscriber callbacks------------//
//-----------------------------------------//

void detect_cb (const std_msgs::Float64MultiArrayConstPtr& detectmsg)
{
	std_msgs::Float64MultiArray detection_sub;
	double dshortest;
	float left, right, center;
	bool shortestL, shortestR, shortestC;

	left = detection_sub.data[0];
	right = detection_sub.data[1];
	center = detection_sub.data[2];

	if (left && right && center < 1)
	{//only find dshortest when detected distance < 1m from UAV.

		bool detect_flag = 1; //On detect_flag to let us know cam detect object less than 1 metre

		//finding where the dshortest distance is
		if (left < center){

		bool shortestL = 1;
		bool shortestR = 0;
		bool shortestC = 0;


		}else if (left < right){

		bool shortestL = 1;
		bool shortestR = 0;
		bool shortestC = 0;


		}else if (right < center){

		bool shortestL = 0;
		bool shortestR = 1;
		bool shortestC = 0;


		}else if (right < left){

		bool shortestL = 0;
		bool shortestR = 1;
		bool shortestC = 0;


		}else if (center < left){

		bool shortestL = 0;
		bool shortestR = 0;
		bool shortestC = 1;


		}else if (center < right){

		bool shortestL = 0;
		bool shortestR = 0;
		bool shortestC = 1;


		}

		//getting only the dshortest distance length from 3 inputs

		if (left < right) dshortest = left;
		if (left < center) dshortest = left;
		if (right < left) dshortest = right;
		if (right < center) dshortest = right;
		if (center < right) dshortest = center;
		if (center < left) dshortest = center;

		//test by cout
		ROS_INFO("shortestL: %s\n", shortestL, "shortestC: %s\n", shortestC, "shortestR: %s\n", shortestR, "dshortest: %s\n", dshortest);

		}
	ros::spin();


}




void RCOut_cb (const mavros_msgs::RCOutConstPtr& rcoutmsg)
{
	ros::Publisher RC_state_now;
	mavros_msgs::RCOut RC_now;
	ROS_INFO("Roll channel: %s", RC_now.channels[1]);
	ROS_INFO("Pitch channel: %s", RC_now.channels[2]);
	ROS_INFO("Throttle channel: %s", RC_now.channels[3]);
	ROS_INFO("Yaw channel: %s", RC_now.channels[4]);

	RC_state_now.publish(RC_now);
	ros::spin();
}

ros::Subscriber detection_sub;
ros::Subscriber RC_state_now;
ros::Publisher CustomRC;

//-----------------------------------------//
//----------------Main---------------------//
//-----------------------------------------//
int main(int argc, char **argv)
{

	ros::init(argc, argv, "reaction");

    ros::NodeHandle nh;


    //-----------------------------------------//
    //----------Assign topic names-------------//
    //-----------------------------------------//
    ros::Subscriber detection_sub = nh.subscribe<std_msgs::Float64MultiArray>("/detection", 60, detect_cb);					//Get distances from camera
    ros::Subscriber RC_state_now = nh.subscribe<mavros_msgs::RCOut>("/mavros_msgs/RCOut", 60, RCOut_cb);//Get current RC values
    ros::Publisher CustomRC = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros_msgs/OverrideRCIn",60);			//Send control input for Collision Avoidance

    SubscribeAndPublishIMU SAPIMU;
    SubscribeAndPublishState SAPState;

    geometry_msgs::PoseStamped pose;
    mavros_msgs::PositionTarget end;

	cout << "Select desired pose, x: \n";
	cin >> end.position.x;

	cout << "Select desired pose, y: \n";
	cin >> end.position.y;

	cout << "Select desired pose, z: \n";
	cin >> end.position.z;

    while(ros::ok())
    {
        do{

        	ros::Rate r(10);
        	double move();
            ros::spinOnce();
            r.sleep();

    	}while((pose.pose.position.x =! end.position.x) && (pose.pose.position.y != end.position.y) && (pose.pose.position.z != end.position.z));

    }

}

//-----------------------------------------//
//-------------END OF MAIN-----------------//
//-----------------------------------------//

////Tuning
//	double Tuning(double P, double I, double D, double Kp, double Ki, double Kd)
//		{
//			cout << "Enter Desired Kp: \n" << Kp << "Enter desired Ki: \n" << Ki << "Enter desired Kd: \n" << Kd << endl;
//			P = Kp;
//			I = Ki;
//			D = Kd;
//			return P, I, D;
//		}

//move UAV
void move(bool detect_flag, bool shortestL,	bool shortestR,	bool shortestC,	double dshortest)
{
	/* RC minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
	 *
	 * Channel 1: Roll input
	 * Channel 2: Pitch input
	 * Channel 3: Throttle input
	 * Channel 4: Yaw input
	 *
	 * Channel value range from 1000 - 2000
	 * 1000 = off
	 * 2000 = full
	 * 800 - 1000 and 2000 - 2200 = dead zone
	 * 1500 = neutral point
	 *
	 *  ===================NOTE===========================
	 * ||Roll: 1000 - 1450 (Left) && 1550 - 2000 (Right) ||
	 * ||Pitch: 1000 - 1450 (Back) && 1550 - 2000 (Front)||
	 * ||Throttle: 1000 (Idle) && 2000 (Max)             ||
	 * ||Yaw: 1000 - 1450 (Left) && 1550 - 2000 (Right)  ||
	 *  ==================================================
	 * mavros_msgs::OverrideRCIn Custom;
	 *
	 * Custom.channels[1] = 1500; //Roll http://ardupilot.org/copter/docs/parameters.html#rcmap-parameters
	 * Custom.channels[2] = 1500; //Pitch
	 * Custom.channels[3] = 0;    //Throttle
	 * Custom.channels[4] = 1500; //Yaw
	 * Custom.channels[5] = 0;
	 * Custom.channels[6] = 0;
	 * Custom.channels[7] = 0;    //Set above 1800 and select value to activate modes
	 * Custom.channels[8] = 0;      from: http://ardupilot.org/copter/docs/parameters.html#rcmap-roll
	 *
        if(share_memory->getOverride()){
            msg_override.channels[0] = share_memory->getRoll();
            msg_override.channels[1] = share_memory->getPitch();
            msg_override.channels[2] = share_memory->getThrottle();
            msg_override.channels[3] = share_memory->getYaw();
            msg_override.channels[4] = 1100;
            msg_override.channels[5] = 1100;
            msg_override.channels[6] = 1100;
            msg_override.channels[7] = 1100;
        }else{
            for(int i = 0; i < 8; i++){
                msg_override.channels[i] = 0;
            }
        }
        if(!stop){
            rc_override_pub.publish(msg_override);
        }
        ros::spinOnce();
	*/

	//Initializing stage
	mavros_msgs::OverrideRCIn Custom;
	mavros_msgs::RCOut State_now;



	if(detect_flag == 1)
	{
		//Initialize neutral point before starting
		Custom.channels[1] = State_now.channels[1];
		Custom.channels[2] = State_now.channels[2];
		Custom.channels[3] = State_now.channels[3];
		Custom.channels[4] = State_now.channels[4];
		Custom.channels[5] = 0;
		Custom.channels[6] = 0;
		Custom.channels[7] = 0;
		Custom.channels[8] = 0;

		ROS_INFO("Obstacle detected! Attempting to avoid obstacle.");

		ros::Duration(1);

	} else{
		//Stop all
		if(!ros::ok())
		{
			for (int i = 0; i <= 8; i++)
				{
					Custom.channels[i] = 0;//Force all modes to 0;
					ROS_INFO("Exiting Avoidance loop, ROS is not running.");

				}
		}

	}


	//Then run Avoidance motion only if dshortest < 1

	if (dshortest >  1.00)
	{
		if(detect_flag == 0)
		{
			while(ros::ok)
			{
				break;//leave avoidance loop
			}

		}
	}else {// reaction

		if(shortestL == 1)
		{//inherent priority where LEFT has obstacle

				do
				{
					Custom.channels[1] = 1600; //Roll right
					Custom.channels[2] = 1500; //No pitch
					Custom.channels[3] = State_now.channels[3]; //Current altitude
					Custom.channels[4] = 1500; //No yaw
					ROS_INFO("Rolling right");
					CustomRC.publish(Custom);
					ros::spinOnce();
					bool detect_flag = 0;
				}while(detect_flag == 1);

			ros::Duration(0.5).sleep(); //Pause for abit


		}else if (shortestR == 1)
		{// RIGHT has obstacle
			while(detect_flag ==1)
			{
				do
				{
					Custom.channels[1] = 1400; //Roll right
					Custom.channels[2] = 1500; //No pitch
					Custom.channels[3] = State_now.channels[3]; //Current altitude
					Custom.channels[4] = 1500; //No yaw
					ROS_INFO("Rolling left");
					CustomRC.publish(Custom);
					ros::spinOnce();
					bool detect_flag = 0;
				}while(detect_flag == 1);
			}

			ros::Duration(0.5).sleep(); //Pause for abit



		}else if (shortestC == 1)
		{//Center detected, move Right
			while(detect_flag ==1)
			{
				do
				{
					Custom.channels[1] = 1600; //Roll right
					Custom.channels[2] = 1500; //No pitch
					Custom.channels[3] = State_now.channels[3]; //Current altitude
					Custom.channels[4] = 1500; //No yaw
					ROS_INFO("Center detected but moving right..");
					CustomRC.publish(Custom);
					ros::spinOnce();
					bool detect_flag = 0;
				}while(detect_flag == 1);
			}

			ros::Duration(0.5).sleep(); //Pause for abit


		}
	}
			ros::spinOnce();

}

//-----------------------------------------//
//-------Determine dshortest distance-------//
//-----------------------------------------//

//void shortestDist(bool shortestL, bool shortestR, bool shortestC, double left, double center, double right, double dshortest)
//{
//
//}

////convert Cartesian to spherical 3D coordinate
//double cartesian_to_spherical(double x, double y, double z)
//{
//	double p(double r, double theta, double phi);
//
//    double r = sqrt(x * x + y * y + z * z);
//    double theta = acos(z / r);
//    double phi = atan2(y, x);
//
//    return p(double r, double theta, double phi);
//}

//void PID(double Error, double new_Error, double P, double I, double D, double output)
//{
//	double secs =ros::Time::now().toSec();
//	double now = secs - prev_Time; //dt
//	geometry_msgs::TwistStamped linear_now;
//	geometry_msgs::Twist linear_goal;
//
//	if (D > 0)
//	{
//		double Error = linear_now.twist.linear.x - linear_goal.linear.x;
//		output = P * new_Error + I * new_Error * now + D * new_Error / now;
//		PID_Publisher.publish(output);
//		ros::spinOnce();
//	}else
//		{
//			ROS_INFO("D cannot be 0, shutting down");
//			return;
//		}
//
//}

