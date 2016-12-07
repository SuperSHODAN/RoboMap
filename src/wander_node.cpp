#include "ros/ros.h"
#include "RoboMap/behavior.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "RoboMap/mapData.h"

void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
geometry_msgs::Point current;

class Wanderer {
public:
	ros::Publisher pub_arbiter;
	ros::NodeHandle nh;
	RoboMap::behavior msg;
	ros::ServiceClient client;
	RoboMap::mapData srv;
	ros::Subscriber sub_l;
	

Wanderer(){
	pub_arbiter = nh.advertise<RoboMap::behavior>("/behavior/wander", 1);
	sub_l = nh.subscribe("robot_pose_ekf/odom_combined", 1, callback);
	client = nh.serviceClient<RoboMap::mapData>("mapData");
}

void create_msg(){
	msg.active = true;

	if(rand() % (100) < 50){
			msg.vel_fw = 0.5;
		} else {
			msg.vel_fw = 0.5;
		}

	if(rand() % 100 < 50){
		if(rand() % 100 < 50){
			msg.vel_turn = -1.0;
		} else {
			msg.vel_turn = 1.0;
		}
	} else {
		msg.vel_turn = 0;
	}
}

void send_msg(){
	pub_arbiter.publish(msg);

	srv.request.latitude = current.x;
    srv.request.longitude = current.y;

	if(client.call(srv)) {
		ROS_INFO("Service call was successful.");
	} else {
		ROS_INFO("Service call has failed.");
	}

	ROS_INFO("send_msg complete.");
}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "wander_node");
	ros::NodeHandle nh;
	ROS_INFO("Starting wander node...");
	ros::Rate loop_rate(30);
	Wanderer w;

	current.x = 0.0;
	current.y = 0.0;
	current.z = 0.0;

	while(ros::ok()) {
		w.create_msg();
		w.send_msg();
		ROS_INFO("sending wander message");

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	current = msg->pose.pose.position;
	ROS_INFO("position retrieved");
}