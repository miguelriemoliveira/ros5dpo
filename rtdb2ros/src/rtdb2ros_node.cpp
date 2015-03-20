//Ros includes
//
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

//Cambada includes
#include "Robot.h"

#include <signal.h>
#include <sys/types.h>
#include <libs/pman/pman.h>
#include "agent/Cambada.h"
#include "pmandefs.h"
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include "rtdb.h"
#include "rtdb_api.h"

/* To test copy paste this to a shell after you launch the node
   rostopic pub /velocity_cmd_agent1 geometry_msgs/Twist "linear:
x: 3.
y: 0.0
z: 0.0
angular:
x: 0.0
y: 0.0
z: 1.0" --rate 1
*/

using namespace std;
using namespace cambada;

Cambada* agent = NULL;
bool EXIT = false;
bool WAIT = true;
char pname[64] = "agent";


int agent_id;
std::string frame_id;
std::string child_frame_id;
nav_msgs::Odometry ro; //a ros format odom
geometry_msgs::PoseStamped ps; //a ros format odom
geometry_msgs::PoseStamped ball_ps; //a ros format odom
ros::Publisher odometry_pub;
ros::Publisher pose_pub;
ros::Publisher ball_pose_pub;

/**
 * @brief Receives a velocity command for agent X and writes that to the rtdb
 *
 * @param msg the velocity command
 */
void velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO("Received velocity command %f %f %f", msg->linear.x, msg->linear.y, msg->angular.z);
    //Cambada coordinate frames is different, so must operate a transform
    //x <- -y  Cambada's x is to the left
    //y <-  x  Cambadas y is in front
    //y = x
	CMD_Vel_SET(-msg->linear.y, msg->linear.x, msg->angular.z,false);
}

/**
 * @brief The timer callback for reading the odometry position from the rtdb
 * and publishing back in ROS format
 *
 */
void timerCallback(const ros::TimerEvent& event)
{
	//Read from rtdb the position and the velocity
	CMD_Pos odom;
	DB_get(agent_id, CMD_POS, (void*)&odom);

	CMD_Vel vel;
	DB_get(agent_id, CMD_VEL, (void*)&vel);

	VisionInfo vi;
	DB_get(agent_id, VISION_INFO, (void*)&vi);

    //ROS_INFO("agent_id = %d ", agent_id);
	//ROS_INFO("Reading VIS_INFO position x=%f y=%f a=%f", vi.position[0], vi.position[1], vi.position[2]);

    //Fill the pose stamped msg and publish
    ps.header.frame_id = "/soccer_field";
    ps.header.stamp = ros::Time::now();
    ps.pose.position.x = vi.sim_agent_position[0];
    ps.pose.position.y = vi.sim_agent_position[1];
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(vi.sim_agent_position[2] + M_PI/2);
    pose_pub.publish(ps);

    //Fill the pose stamped msg and publish
    ball_ps.header.frame_id = "/soccer_field";
    ball_ps.header.stamp = ros::Time::now();
    ball_ps.pose.position.x = vi.sim_ball_position[0];
    ball_ps.pose.position.y = vi.sim_ball_position[1];
    ball_pose_pub.publish(ball_ps);

	//ROS_INFO("Reading odom position x=%f y=%f a=%f", odom.px, odom.py, odom.pa);
	//ROS_INFO("Reading odom dposition x=%f y=%f a=%f", odom.dx, odom.dy, odom.da);
	//ROS_INFO("Reading odom velocity x=%f y=%f a=%f", vel.vx, vel.vy, vel.va);

    //Convert back to Cambada's coordinate frame
    //double ox = odom.py;
    //double oy = -odom.px;

       
	////Publish ros odom message
	//ro.header.stamp = ros::Time::now(); //TODO discuss with Heber about this
	//ro.header.frame_id = frame_id;
	//ro.child_frame_id = child_frame_id;
    //double x = odom.py;
    //double y = -odom.px;
	//ro.pose.pose.position.x = x; //cos(odom.pa)*x + sin(odom.pa)*y;
	//ro.pose.pose.position.y = y; //-sin(odom.pa)*x + cos(odom.pa)*y;
	////ro.pose.pose.position.x = odom.py;
	////ro.pose.pose.position.y = -odom.px;
    ////geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(odom.pa+M_PI/2);
    //geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(odom.pa);
	//ro.pose.pose.orientation = q;
	////ro.pose.pose.orientation.z = odom.pa;
	//ro.twist.twist.linear.x = vel.vx;
	//ro.twist.twist.linear.y = vel.vy;
	//ro.twist.twist.angular.z = vel.va;

	//odometry_pub.publish(ro);	

	//static tf::TransformBroadcaster br;
	//tf::Transform transform;
	//transform.setOrigin(tf::Vector3(odom.px, odom.py, 0.0));
	//tf::Quaternion qtf;
	//qtf.setRPY(0, 0, odom.pa);
	//transform.setRotation(qtf);
	//br.sendTransform(tf::StampedTransform(transform, ro.header.stamp, frame_id, child_frame_id));

}

/**
 * @brief The main function
 */
int main(int argc, char **argv)
{

	//Init rtdb
    if( DB_init() == -1 ) {
        CMD_Vel_SET(0.0,0.0,0.0,false);
        CMD_Grabber_SET(0);
        fprintf(stderr,"ERROR: main: DB_INIT failed\n");
        exit(EXIT_FAILURE);
    }

	//Get agent name by reading environment variable AGENT
	//Use export AGENT=2 to set AGENT 2
	strcat(pname, getenv("AGENT"));
    //cout << "pname=" << pname << endl;
	agent_id = Whoami();

	//Initialize the ROS node
	ros::init(argc, argv, "rtdb2ros_node");
	ros::NodeHandle n;

	//Compute the frame_id and child frame_id names
	frame_id = "/odom_";
	frame_id.append(pname);
	child_frame_id = "/base_footprint_";
	child_frame_id.append(pname);

    //Configure the publisher 
	std::string pose_stamped_topic = "pose_agent";
	pose_pub = n.advertise<geometry_msgs::PoseStamped>(pose_stamped_topic, 100);

    //Configure the publisher 
	std::string ball_pose_stamped_topic = "pose_ball";
	ball_pose_pub = n.advertise<geometry_msgs::PoseStamped>(ball_pose_stamped_topic, 100);



	//Configure the publisher of odometry
	std::string odom_topic = "odom";
	odometry_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 100);

	//Configure subscriber to velocity_cmd
	std::string velocity_cmd_topic = "velocity_cmd";
	ros::Subscriber sub = n.subscribe(velocity_cmd_topic, 100, velocityCommandCallback);

	//Configure a timer callback that regularly reads the odometry position
	//ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);
	ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);
    //std::

    std::string node_name = ros::this_node::getName();


	ROS_INFO("Starting %s", node_name.c_str());
	ROS_INFO("Agent id %d", agent_id);
	ROS_INFO("Subscribing to %s", velocity_cmd_topic.c_str());
	ROS_INFO("frame_id is %s and child_frame_id is %s", frame_id.c_str(), child_frame_id.c_str());


	//Start spinning
	ros::spin();

	return 0;
}
