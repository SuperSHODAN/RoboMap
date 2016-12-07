#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/tf.h"
#include "RoboMap/behavior.h"
#include <cmath>
#include <queue>
#include <deque>
#include "angles/angles.h"

void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void FixTurn();
void FixVel();
void FindFix();
void FindTarget();

RoboMap::behavior msg;
double yaw_angle;
double target_angle;

double error_turn;
double derivative_turn;
double integral_turn = 0;
double old_error_turn = 0;
double turn_p_gain = 0.15;
double turn_i_gain = 0.05;
double turn_d_gain = 0.05;
double turn_power;

std::deque<double> old_error_vel;
double distance;
double derivative_vel;
double integral_vel = 0;
double old_distance = 0;
double vel_p_gain = 0.1;
double vel_i_gain = 0.015;
double vel_d_gain = 0.05;
double vel_power;

double localMap[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
double baseX = -1.0;
double baseY = -1.0;

geometry_msgs::Point current;
geometry_msgs::Point target;

int main(int argc, char** argv) {
	ros::init(argc, argv, "seek_node");
	ros::NodeHandle nh;
	ROS_INFO("Starting seeker node...");

	ros::Rate loop_rate(30);

	ros::Subscriber sub_l = nh.subscribe("robot_pose_ekf/odom_combined", 1, callback);

    ros::Publisher pub_arbiter = nh.advertise<RoboMap::behavior>("/behavior/seek", 1);

    ros::ServiceClient client = nh.serviceClient<RoboMap::mapData>("mapData");
    RoboMap::mapData srv;

    FindTarget();

    for(int i = 0; i < 9; i++) {
    	old_error_vel.push_front(0.0);
    }

	while(ros::ok()) {

		FindFix();

		pub_arbiter.publish(msg);

		/*ROS_INFO("Turn Power: %f", turn_power);
		ROS_INFO("Turn Error: %f", error_turn);
		ROS_INFO("Turn Integral: %f", integral_turn);
		ROS_INFO("Turn Derivative: %f", derivative_turn);*/

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    /*ROS_INFO("robot_pose_ekf:");
    ROS_INFO(" x: %f", msg->pose.pose.orientation.x);
    ROS_INFO(" y: %f", msg->pose.pose.orientation.y);
    ROS_INFO(" z: %f", msg->pose.pose.orientation.z);
    ROS_INFO(" w: %f", msg->pose.pose.orientation.w);*/

    /*ROS_INFO("current:");
    ROS_INFO(" x: %f", msg->pose.pose.position.x);
    ROS_INFO(" y: %f", msg->pose.pose.position.y);
    ROS_INFO(" z: %f", msg->pose.pose.position.z);
    ROS_INFO("target: ");
    ROS_INFO(" x: %f", target.x);
    ROS_INFO(" y: %f", target.y);
    ROS_INFO(" z: %f", target.z);*/
    

    current = msg->pose.pose.position;

    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    yaw_angle = tf::getYaw(pose.getRotation());
    target_angle = std::atan2(target.y - current.y, target.x - current.x);

    distance = sqrt(pow(std::abs(current.x - target.x), 2) + pow(std::abs(current.y - target.y), 2));

    /*ROS_INFO("distance: %f", distance);

    ROS_INFO(" yaw: %f", yaw_angle);
    ROS_INFO(" ");
    ROS_INFO("target_angle: %f", target_angle);
    ROS_INFO(" ");*/

}

void FindTarget () {

	srv.request.latitude = current.x;
    srv.request.longitude = current.y;

	if(client.call(srv)) {
		ROS_INFO("Map received from service call.");
		baseX = srv.response.latitude;
		baseY = srv.response.longitude;
		int k = 0;
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				localMap[i][j] = srv.response.intensity[k];
				k++;
			}
		}
	} else {
		ROS_INFO("Service call to retrieve map has failed.");
	}

	double largest = 0;
	int largestX = 0;
	int largestY = 0;
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			if(localMap[i][j] > largest){
				largest = localMap[i][j];
				largestX = i;
				largestY = j;
			}
		}
	}

	target.x = baseX + largestX;
	target.y = baseY + largestY;

	ROS_INFO("largestX: %i", largestX);
	ROS_INFO("largestY: %i", largestY);

}

void FindFix () {

	if(std::abs(angles::shortest_angular_distance(yaw_angle, target_angle)) >= 0.15 && (distance >= 0.1)) {
		FixTurn();
	} else if (distance >= 0.1) {
		integral_turn = 0;
		FixVel();
	} else {
		/*integral_vel = 0;*/
		msg.vel_fw = 0;
		msg.vel_turn = 0;
		msg.active = true;
		FindTarget();
	}

	/*ROS_INFO("msg: ");
	ROS_INFO("vel_fw: %f", msg.vel_fw);
	ROS_INFO("vel_turn: %f", msg.vel_turn);

	ROS_INFO("shortest angular distance: %f", angles::shortest_angular_distance(yaw_angle, target_angle));*/
}

void FixTurn () {

	error_turn = std::abs(yaw_angle - target_angle);
	integral_turn = integral_turn + (error_turn * (1.0/30.0));
	derivative_turn = (error_turn - old_error_turn)/ (1.0/30.0);

	turn_power = error_turn * turn_p_gain + integral_turn * turn_i_gain + derivative_turn * turn_d_gain;

	old_error_turn = error_turn;
	msg.vel_fw = 0.075;
	msg.active = true;

	if(angles::shortest_angular_distance(yaw_angle, target_angle) <= -0.2){
		msg.vel_turn = turn_power * -1;
	} else {
		msg.vel_turn = turn_power;
	}
}

void FixVel () {
	msg.vel_turn = 0;
	msg.active = true;

	old_error_vel.push_front(distance);
	old_error_vel.pop_back();
	
	/*integral_vel = integral_vel + (distance * (1.0/30.0));*/
	integral_vel = std::accumulate(old_error_vel.begin(), old_error_vel.begin() + 10, 0);
	derivative_vel = (distance - old_distance)/(1.0/30.0);

	vel_power = distance * vel_p_gain + integral_vel * vel_i_gain + derivative_vel * vel_d_gain;

	msg.vel_fw = vel_power;
	old_distance = distance;
}