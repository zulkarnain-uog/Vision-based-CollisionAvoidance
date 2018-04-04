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
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Accel.h"
#include "mavros/mavros.h"
#include "mavros/frame_tf.h"
#include "mavros/px4_custom_mode.h"
#include "mavros_msgs/OverrideRCIn.h"
#include <ros/console.h>
#include <urdf/model.h>
#include <mavros_msgs/RCOut.h>
#include <sensor_msgs/JointState.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <stdbool.h>
#include "mavros_msgs/WaypointPull.h"
#include "mavros_msgs/WaypointPush.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/PositionTarget.h"
#include "std_msgs/MultiArrayDimension.h"

using namespace std;

double prev_Time = ros::Time::now().toSec();

//-----------------------------------------//
//---------------Classes-------------------//
//-----------------------------------------//
void move(double Tuning, double double PID, double detection, double shortestL, double shortestR, double shortestC, double left, double center, double right, double dshortest, double shortestDist);
void PID(double Error, double new_Error, double P, double I, double D);
void shortestDist(double shortestL, double shortestR, double shortestC, double left, double center, double right, double dshortest);
void cartesian_to_spherical(double x, double y, double z);
void PID_RCIn(double OverrideRCIn_msg, double OverrideRCIn, double PID);
void react(double dshortest, double sensor_msgs);
//void Tuning (double P, double I, double D, double Kp, double Ki, double Kd);


//-----------------------------------------//
//----------------Main---------------------//
//-----------------------------------------//
int main(int argc, char **argv)
{
	ros::Time prev_Time;

	ros::init(argc, argv, "reaction");

    ros::NodeHandle nh;


    //-----------------------------------------//
    //----------Assign topic names-------------//
    //-----------------------------------------//

    ros::Subscriber detection = nh.subscribe<std_msgs::MultiArrayDimension>("detection", 60);					//Get distances from camera
//    ros::Publisher servo_state = nh.advertise<sensor_msgs::JointState>("servo_state_publisher",10);
    ros::Subscriber current_state = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);	//Get current UAV status
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);	//Send goal pose to UAV
    ros::Subscriber IMU = nh.subscribe<sensor_msgs::Imu>("IMU_reading", 50);						//Get IMU status and values
    ros::Publisher Custom = nh.advertise<mavros_msgs::OverrideRCIn>("Custom_controls",50);			//Send control input for Collision Avoidance
    ros::Subscriber RC_state_now = nh.subscribe<mavros_msgs::RCOut>("Current RC channel states", 50);//Get current RC values
    ros::Subscriber goal = nh.subscribe<mavros_msgs::PositionTarget>("Goal_pose", 10);				//Get Current pose
    ros::Rate rate(50);


    	// wait for FCU connection

        while(ros::ok() && !current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }

    	geometry_msgs::PoseStamped pose;
        	pose.pose.position.x = 0;
        	pose.pose.position.y = 0;
        	pose.pose.position.z = 2;

        	//send a few set-points before starting
        	for(int i = 100; ros::ok() && i > 0; --i)
        	{
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        	}

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = 1;

        ros::Time last_request = ros::Time::now();


        while(ros::ok()){
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");

                        //Request if tuning is desired
                        bool Tuning = 1; //1 for yes, 0 for no

                        mavros_msgs::PositionTarget goal;
                        cout << "Select desired pose, x: \n" << goal.position.x;
                        cout << "Select desired pose, y: \n" << goal.position.y;
                        cout << "Select desired pose, z: \n" << goal.position.z;

                        if (!Tuning == 0)
                          {
                            do{
                            	if(dshortest < 1)
                            	{
                            		bool reaction_flag = 1;
                            		double move();
                            	}else
                            	{
                            		bool reaction_flag = 0;
                            	}
                            }while((pose.pose.position.x =! goal.position.x) && (pose.pose.position.y != goal.position.y) && (pose.pose.position.z != goal.position.z));

                          }else {
                               double Tuning();
                               break;
                          }
                    }
                    last_request = ros::Time::now();
                }
            }

            local_pos_pub.publish(pose);

            ros::spinOnce();
            rate.sleep();
        }

    	bool Tuning = 0;
        while (ros::ok)
        {

        	//insert autotune line here

        }

        return 0;
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
double move(double Tuning, double PID, double detection, double shortestL, double shortestR, double shortestC, double left, double center, double right, double dshortest, double shortestDist)
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
	std_msgs::MultiArrayDimension Obsdistance;
	ros::Time duration = ros::Time::now() + ros::Duration(1);


	if(Obsdistance.flag == 1)
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
					break;
				}
		}

	}


	//Then run Avoidance motion only if dshortest < 1

	if (dshortest >  1.00)
	{
		if(Obsdistance.flag == 0)
		{
			while(ros::ok)
			{
				break;//leave avoidance loop
			}

		}
	}else {// reaction

		if(shortestL == 1)
		{//inherent priority where LEFT has obstacle
			while(Obsdistance.flag ==1)
			{
				do
				{
					Custom.channels[1] = 1600; //Roll right
					Custom.channels[2] = 1500; //No pitch
					Custom.channels[3] = State_now.channels[3]; //Current altitude
					Custom.channels[4] = 1500; //No yaw
					ros::spinOnce();
				}while(ros::Time::now() != ros::Time::now() + duration);
			}

			ros::Duration(0.5).sleep(); //Pause for abit
			break;

		}else if (shortestR == 1)
		{// RIGHT has obstacle
			while(Obsdistance.flag ==1)
			{
				do
				{
					Custom.channels[1] = 1400; //Roll right
					Custom.channels[2] = 1500; //No pitch
					Custom.channels[3] = State_now.channels[3]; //Current altitude
					Custom.channels[4] = 1500; //No yaw
					ros::spinOnce();
				}while(duration);
			}

			ros::Duration(0.5).sleep(); //Pause for abit
			break;


		}else if (shortestC == 1)
		{//Center detected, move Right
			while(Obsdistance.flag ==1)
			{
				do
				{
					Custom.channels[1] = 1600; //Roll right
					Custom.channels[2] = 1500; //No pitch
					Custom.channels[3] = State_now.channels[3]; //Current altitude
					Custom.channels[4] = 1500; //No yaw
					ros::spinOnce();
				}while(duration);
			}

			ros::Duration(0.5).sleep(); //Pause for abit
			break;

		}
	}
			ros::spinOnce();
			break;
}

//-----------------------------------------//
//-------Determine dshortest distance-------//
//-----------------------------------------//

double shortestDist(double shortestL, double shortestR, double shortestC, double left, double center, double right, double dshortest){

bool reaction_flag = 0; //Flag to signal when code is running avoidance decision making algo

Obsdistance detection_now;

if (detection_now.left && detection_now.right && detection_now.center < 1){//only find dshortest when detected distance < 1m from UAV.

bool reaction_flag = 1;
	//finding where the dshortest distance is
if (detection_now.left < detection_now.center){

bool shortestL = 1;
bool shortestR = 0;
bool shortestC = 0;


}else if (detection_now.left < detection_now.right){

bool shortestL = 1;
bool shortestR = 0;
bool shortestC = 0;


}else if (detection_now.right < detection_now.center){

bool shortestL = 0;
bool shortestR = 1;
bool shortestC = 0;


}else if (detection_now.right < detection_now.left){

bool shortestL = 0;
bool shortestR = 1;
bool shortestC = 0;


}else if (detection_now.center < detection_now.left){

bool shortestL = 0;
bool shortestR = 0;
bool shortestC = 1;


}else if (detection_now.center < detection_now.right){

bool shortestL = 0;
bool shortestR = 0;
bool shortestC = 1;

}else{

bool shortestL = 0;
bool shortestR = 0;
bool shortestC = 0;

}

//getting only the dshortest distance length from 3 inputs
if (detection_now.left < detection_now.right) dshortest = left;
if (detection_now.left < detection_now.center) dshortest = left;
if (detection_now.right < detection_now.left) dshortest = right;
if (detection_now.right < detection_now.center) dshortest = right;
if (detection_now.center < detection_now.right) dshortest = center;
if (detection_now.center < detection_now.left) dshortest = center;

//test by cout
cout << "shortestL : /n" << shortestL << "shortestC : \n" << shortestC << "shortestR : \n"
		<< shortestR << "dshortest : " << dshortest << endl;

cout << "dshortest : " << dshortest << endl;
reaction_flag = 0;

}
return reaction_flag;
}

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

/*
//PWM control of motor
class ServoDescription {
public:
	std::string joint_name;
	float joint_lower;
	float joint_upper;

	size_t rc_channel;

	uint16_t rc_min;
	uint16_t rc_max;
	uint16_t rc_trim;
	uint16_t rc_dz;
	bool rc_rev;

	ServoDescription() :
		joint_name{},
		joint_lower(-M_PI/4),
		joint_upper(M_PI/4),
		rc_channel(0),
		rc_min(1000),
		rc_max(2000),
		rc_trim(1500),
		rc_dz(0),
		rc_rev(0)
	{ };

	ServoDescription(std::string joint_name_, double lower_, double upper_,
			int channel_,
			int min_, int max_, int trim_, int dz_,
			bool rev_) :
		joint_name(joint_name_),
		joint_lower(lower_),
		joint_upper(upper_),
		rc_channel(channel_),
		rc_min(min_),
		rc_max(max_),
		rc_trim(trim_),
		rc_dz(dz_),
		rc_rev(rev_)
	{ };

	/**
	 * Normalization code taken from PX4 Firmware
	 * src/modules/sensors/sensors.cpp Sensors::rc_poll() line 1966
	 */
/*	inline float normalize(uint16_t pwm) {
		// 1) fix bounds
		pwm = std::max(pwm, rc_min);
		pwm = std::min(pwm, rc_max);

		// 2) scale around mid point
		float chan;
		if (pwm > (rc_trim + rc_dz)) {
			chan = (pwm - rc_trim - rc_dz) / (float)(rc_max - rc_trim - rc_dz);
		}
		else if (pwm < (rc_trim - rc_dz)) {
			chan = (pwm - rc_trim + rc_dz) / (float)(rc_trim - rc_min - rc_dz);
		}
		else {
			chan = 0.0;
		}

		if (rc_rev)
			chan *= -1;

		if (!std::isfinite(chan)) {
			ROS_DEBUG("SSP: not finite result in RC%zu channel normalization!", rc_channel);
			chan = 0.0;
		}

		return chan;
	}

	float calculate_position(uint16_t pwm) {
		float channel = normalize(pwm);

		// not sure should i differently map -1..0 and 0..1
		// for now there arduino map() (explicit)
		float position = (channel + 1.0) * (joint_upper - joint_lower) / (1.0 + 1.0) + joint_lower;

		return position;
	}
};

class ServoStatePublisher {
public:
	ServoStatePublisher() :
		nh()
	{
		ros::NodeHandle priv_nh("~");

		XmlRpc::XmlRpcValue param_dict;
		priv_nh.getParam("", param_dict);

		ROS_ASSERT(param_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);

		urdf::Model model;
		model.initParam("robot_description");
		ROS_INFO("SSP: URDF robot: %s", model.getName().c_str());

		for (auto &pair : param_dict) {
			ROS_DEBUG("SSP: Loading joint: %s", pair.first.c_str());

			// inefficient, but easier to program
			ros::NodeHandle pnh(priv_nh, pair.first);

			bool rc_rev;
			int rc_channel, rc_min, rc_max, rc_trim, rc_dz;

			if (!pnh.getParam("rc_channel", rc_channel)) {
				ROS_ERROR("SSP: '%s' should provice rc_channel", pair.first.c_str());
				continue;
			}

			pnh.param("rc_min", rc_min, 1000);
			pnh.param("rc_max", rc_max, 2000);
			if (!pnh.getParam("rc_trim", rc_trim)) {
				rc_trim = rc_min + (rc_max - rc_min) / 2;
			}

			pnh.param("rc_dz", rc_dz, 0);
			pnh.param("rc_rev", rc_rev, 0);

			auto joint = model.getJoint(pair.first);
			if (!joint) {
				ROS_ERROR("SSP: URDF: there no joint '%s'", pair.first.c_str());
				continue;
			}
			if (!joint->limits) {
				ROS_ERROR("SSP: URDF: joint '%s' should provide <limit>", pair.first.c_str());
				continue;
			}

			double lower = joint->limits->lower;
			double upper = joint->limits->upper;

			servos.emplace_back(pair.first, lower, upper, rc_channel, rc_min, rc_max, rc_trim, rc_dz, rc_rev);
			ROS_INFO("SSP: joint '%s' (RC%d) loaded", pair.first.c_str(), rc_channel);
		}

		rc_out_sub = nh.subscribe("rc_out", 10, &ServoStatePublisher::rc_out_cb, this);
		joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
	}

	void spin() {
		if (servos.empty()) {
			ROS_WARN("SSP: there nothing to do, exiting");
			return;
		}

		ROS_INFO("SSP: Initialization done. %zu joints served", servos.size());
		ros::spin();
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber rc_out_sub;
	ros::Publisher joint_states_pub;

	std::list<ServoDescription> servos;

	void rc_out_cb(const mavros_msgs::RCOut::ConstPtr &msg) {
		if (msg->channels.empty())
			return;		// nothing to do

		auto states = boost::make_shared<sensor_msgs::JointState>();
		states->header.stamp = msg->header.stamp;

		for (auto &desc : servos) {
			if (!(desc.rc_channel != 0 && desc.rc_channel <= msg->channels.size()))
				continue;	// prevent crash on servos not in that message

			uint16_t pwm = msg->channels[desc.rc_channel - 1];
			if (pwm == 0 || pwm == UINT16_MAX)
				continue;	// exclude unset channels

			states->name.emplace_back(desc.joint_name);
			states->position.emplace_back(desc.calculate_position(pwm));
		}

		joint_states_pub.publish(states);
	}
};
*/
