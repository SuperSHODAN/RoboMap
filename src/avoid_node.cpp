#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "RoboMap/behavior.h"

void left_callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
void right_callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
void check_path();

bool left_path_blocked = false;
bool right_path_blocked = false;
double left_laser_value;
double right_laser_value;
RoboMap::behavior msg;

int main(int argc, char** argv) {
	ros::init(argc, argv, "avoid_node");
	ros::NodeHandle nh;
	ROS_INFO("Starting evasion node...");
	ros::Rate loop_rate(30);

	ros::Subscriber left_laser = nh.subscribe("irobot/left_distance_scan", 1, left_callback_laser);
	ros::Subscriber right_laser = nh.subscribe("irobot/right_distance_scan", 1, right_callback_laser);

	ros::Publisher pub_arbiter = nh.advertise<RoboMap::behavior>("behavior/avoid", 1);

	while(ros::ok()) { 	
		check_path();

		pub_arbiter.publish(msg);
		left_path_blocked = false;
		right_path_blocked = false;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void left_callback_laser (const sensor_msgs::LaserScan::ConstPtr& msg) {
	ROS_INFO("Angle Min: %f Max: %f Inc: %f", msg->angle_min, msg->angle_max, msg->angle_increment);
	ROS_INFO("Angle Min: %f Max: %f", msg->range_min, msg->range_max);
	for (int i = 0; i < msg->ranges.size(); i++){
		if (msg->ranges[i] < 1.10) {
			left_path_blocked = true;
			left_laser_value = msg->ranges[i];
		}
	}

}

void right_callback_laser (const sensor_msgs::LaserScan::ConstPtr& msg) {
	ROS_INFO("Angle Min: %f Max: %f Inc: %f", msg->angle_min, msg->angle_max, msg->angle_increment);
	ROS_INFO("Angle Min: %f Max: %f", msg->range_min, msg->range_max);
	for (int i = 0; i < msg->ranges.size(); i++){
		if (msg->ranges[i] < 1.10) {
			right_path_blocked = true;
			right_laser_value = msg->ranges[i];
		}
	}

}

void check_path () {
	msg.vel_fw = 0;
	if(left_path_blocked && right_path_blocked) {
		msg.active = true;
		if(left_laser_value < right_laser_value) {
			msg.vel_turn = -2;
			msg.vel_fw = -1;
		} else {
			msg.vel_turn = 2;
			msg.vel_fw = -1;
		}
	} else if(left_path_blocked) {
		msg.active = true;
		msg.vel_turn = -1;
	} else if (right_path_blocked) {
		msg.active = true;
		msg.vel_turn = 1;
	} else {
		msg.active = false;
		msg.vel_turn = 0;
	}
	ROS_INFO("left blocked (%s)", left_path_blocked ? "yes" : "no");
	ROS_INFO("right blocked (%s)", right_path_blocked ? "yes" : "no");
}