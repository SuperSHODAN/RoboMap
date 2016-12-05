#include "ros/ros.h"
#include "class_excercises/behavior.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/tf.h"

class Detect{
	public:
	ros::Subscriber sub_left_scan;
	ros::Subscriber sub_right_scan;
	ros::Subscriber sub_ekf;
	ros::Publisher pub_detect;
	class_excercises::behavior msg_bh;



	ros::NodeHandle nh;
	bool left_blocked = false;
	bool right_blocked = false;

	double yaw_angle;
	geometry_msgs::Point current_location;
	geometry_msgs::Point target_location;

	Detect(){ 
		this->pub_detect = nh.advertise<class_excercises::behavior>("behavior/detect", 1);
		this->sub_left_scan = nh.subscribe("irobot/left_distance_scan", 1, &Detect::callback_left_distance_scan, this);
		this->sub_right_scan = nh.subscribe("irobot/right_distance_scan", 1, &Detect::callback_right_distance_scan, this);
        this->sub_ekf = nh.subscribe("robot_pose_ekf/odom_combined", 1, &Detect::cb_ekf, this);
	}
  
	void process_detect();
	void callback_left_distance_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
	void callback_right_distance_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
	void cb_ekf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
	void triangulate(double distance, bool left);
};

void Detect::process_detect(){

	left_blocked = false;
	right_blocked = false;
};

void Detect::callback_left_distance_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
	// ROS_INFO("Angle Min: %f Max %f Inc %f", msg->angle_min, msg->angle_max, msg->angle_increment);
	// ROS_INFO("Range Min: %f Max %f", msg->range_min, msg->range_max);
	for (int i = 0; i < msg->ranges.size(); i++){
		// ROS_INFO("distance: %f", msg->ranges[i]);
		if (msg->ranges[i] < 1.0) {
			left_blocked = true;
			triangulate(msg->ranges[i], true);
		}
		else{
			left_blocked    = false;
		}
	}
}

void Detect::callback_right_distance_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
	for (int i = 0; i < msg->ranges.size(); i++){
		// ROS_INFO("distance: %f", msg->ranges[i]);
		if (msg->ranges[i] < 1.0) {
			triangulate(msg->ranges[i], false);
			right_blocked = true;
		}
		else{
			right_blocked   = false;
		}
		
	}
}

void Detect::triangulate(double distance, bool left){
	float adj_side = distance * std::sin(yaw_angle);
	float op_side =  distance * std::cos(yaw_angle);

	if (yaw_angle > (0.5f * 3.14f)){
		ROS_INFO("Quad 3");	
		target_location.x = current_location.x - op_side;
		target_location.y = current_location.y - adj_side;
	}
	else if(yaw_angle > 0 && yaw_angle < (0.5f * 3.14f)){
		ROS_INFO("Quad 2");	
		target_location.x = current_location.x + op_side;
		target_location.y = current_location.y - adj_side;
	}
	else if (yaw_angle < 0 && yaw_angle > (-0.5f * 3.14f)){
		ROS_INFO("Quad 1");	
		target_location.x = current_location.x + op_side;
		target_location.y = current_location.y + adj_side;
	}
	else if (yaw_angle < (-0.5f * 3.14f)){
		ROS_INFO("Quad 4");	
		target_location.x = current_location.x - op_side;
		target_location.y = current_location.y + adj_side;
	}

	if (left){
		ROS_INFO("LEFT SENSOR");
	}
	else{
		ROS_INFO("RIGHT SENSOR");
	}

	ROS_INFO("distance: %f", distance);
	ROS_INFO("angle: %f", yaw_angle);
	ROS_INFO("Target location x: %f, y: %f", target_location.x, target_location.y);
	ROS_INFO("Current location x: %f, y: %f", current_location.x, current_location.y);

	msg_bh.target_x = target_location.x;
	msg_bh.target_y = target_location.y;

	pub_detect.publish(msg_bh);
}

void Detect::cb_ekf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);

    current_location = msg->pose.pose.position;

    yaw_angle = tf::getYaw(pose.getRotation());
    // target_angle = std::atan2(target.y - current.y, target.x - current.x);
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "detect");
	ROS_INFO("Starting detect node...");

	ros::NodeHandle nh;
	ros::Rate loop_rate(40);

	Detect detectNode;

	while(ros::ok()) {
		detectNode.process_detect();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}