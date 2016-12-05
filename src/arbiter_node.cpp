#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "robotics_class/behavior.h"
#include <functional>
#include <queue>
#include <vector>

//define priority of behaviors here
#define PRIORITY_SEEK 0
#define PRIORITY_DETECT 1

void callback_seek(const robotics_class::behavior::ConstPtr& msg);
void callback_detect(const robotics_class::behavior::ConstPtr& msg);
geometry_msgs::Twist msg_move;
robotics_class::behavior msg_bh;
ros::Publisher pub_vel;

//define comparison operator here
bool compare_priorities(std::pair<int, robotics_class::behavior> a,
             std::pair<int, robotics_class::behavior> b) {
    return a.first < b.first;
}

//create priority queue
std::priority_queue< std::pair<int, robotics_class::behavior>,
            std::vector<std::pair<int, robotics_class::behavior> >,
            decltype(&compare_priorities)> behavior_queue{compare_priorities};

//translate behavior message to twist
void move_robot(robotics_class::behavior& msg) {
    geometry_msgs::Twist msg_move;
    msg_move.linear.x = msg.vel_fw;
    msg_move.angular.z = msg.vel_turn;
    pub_vel.publish(msg_move);
}

void stop_robot() {
    geometry_msgs::Twist msg_move;
    msg_move.linear.x = 0;
    msg_move.angular.z = 0;
    pub_vel.publish(msg_move);
}

//process all behavior messages
void process_behaviors() {
    if (!behavior_queue.empty()) {
        robotics_class::behavior priority_msg = behavior_queue.top().second;
        move_robot(priority_msg);
        while(behavior_queue.size() > 0) {
            //C++ Priority Queue has no clear function,
            //so we loop to pop off the old messages
            behavior_queue.pop();
        }
    }
    else {
        stop_robot();
    }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "arbiter_node");
	ROS_INFO("Starting arbiter node.");
	ros::NodeHandle nh;
	ros::Rate loop_rate(30);

	ros::Subscriber sub_seek = nh.subscribe("behavior/seek", 1, callback_seek);
	ros::Subscriber sub_avoid = nh.subscribe("behavior/detect", 1, callback_detect);

	pub_vel = nh.advertise<geometry_msgs::Twist>("/irobot/cmd_vel", 1);



	while(ros::ok()) {
		
		process_behaviors();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

//callbacks for behaviors
void callback_seek (const robotics_class::behavior::ConstPtr& msg) {
	if (msg->active) {
        behavior_queue.push(std::pair<int, robotics_class::behavior>(PRIORITY_SEEK, *msg));
    }
    ROS_INFO("Arbiter: Seek(%s) Fw: %.1f Turn: %.1f", msg->active ? "on" : "off", msg->vel_fw, msg->vel_turn);
}

void callback_detect (const robotics_class::behavior::ConstPtr& msg) {
	if (msg->active) {
        behavior_queue.push(std::pair<int, robotics_class::behavior>(PRIORITY_DETECT, *msg));
    }
    ROS_INFO("Arbiter: Detect(%s) Fw: %.1f Turn: %.1f", msg->active ? "on" : "off", msg->vel_fw, msg->vel_turn);
}