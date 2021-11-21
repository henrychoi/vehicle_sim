#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

using namespace std;

class DbwNode {
	typedef DbwNode Self;
    ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher pub_;
    int steer_axis_, throttle_axis_, deadman_btn_;
	float steer_gain_, throttle_gain_;

	void onInput(const sensor_msgs::Joy::ConstPtr&);

public:
	DbwNode();
};

DbwNode::DbwNode()
: nh_("dbw")
, sub_(nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Self::onInput, this))
, pub_(nh_.advertise<geometry_msgs::Twist>("cmd_vel",1))
{
	assert(nh_.param("throttle_axis", throttle_axis_, 1));
	assert(nh_.param("steer_axis", steer_axis_, 0));
	assert(nh_.param("deadman_btn", deadman_btn_, 4));
	assert(nh_.param("throttle_gain", throttle_gain_, 1.f));
	assert(nh_.param("steer_gain", steer_gain_, 1.f));

	// ROS_ERROR("steer_gain %f", steer_gain_);
}

void DbwNode::onInput(const sensor_msgs::Joy::ConstPtr& input) {
	if (steer_axis_ >= input->axes.size()) {
		ROS_ERROR("Precondition violation: invalid steer_axis");
		return;
	}
	if (throttle_axis_ >= input->axes.size()) {
		ROS_ERROR("Precondition violation: invalid throttle_axis");
		return;
	}

	ROS_DEBUG("onInput %d %.2f %.2f", input->buttons[deadman_btn_]
		, input->axes[steer_axis_], input->axes[throttle_axis_]);
	if (input->buttons[deadman_btn_]) {
	geometry_msgs::Twist vel;
		vel.angular.z = steer_gain_ * input->axes[steer_axis_];
		vel.linear.x = throttle_gain_ * input->axes[throttle_axis_];
	pub_.publish(vel); 
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "dbw");
	DbwNode node;
	ros::spin();

	return 0;
}
