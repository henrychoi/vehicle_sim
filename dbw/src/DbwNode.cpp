#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <hcpath/parkAction.h>
#include <boost/circular_buffer.hpp>
#include <numeric>

using namespace std;

class DbwNode {
	typedef DbwNode Self;
    ros::NodeHandle _nh;
	ros::Subscriber joy_sub_;
	void onInput(const sensor_msgs::Joy::ConstPtr&);

    ros::Subscriber joint_sub_;
	void onJointState(const sensor_msgs::JointState::ConstPtr &state);
    double _dr = 0, _dl = 0; // front wheel angles

	ros::Publisher pub_;
	actionlib::SimpleActionClient<hcpath::parkAction> ac_;

    int steer_axis_, throttle_axis_, tank_axis_, deadman_btn_, tank_btn_, x_btn_;
	float steer_gain_, throttle_gain_;

	static constexpr size_t kXbtnBufferSize = 5, kTankBtnBufferSize = 5;
	boost::circular_buffer<int8_t> xQ_;
	bool x_btn_state = false;

public:
	DbwNode();
};

DbwNode::DbwNode()
: _nh("dbw")
, joy_sub_(_nh.subscribe<sensor_msgs::Joy>("joy", 10, &Self::onInput, this))
, joint_sub_(_nh.subscribe("/autoware_gazebo/joint_states", 1, &Self::onJointState, this))
, pub_(_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1))
, ac_("/path", false)// true causes the client to spin its own thread
, xQ_(kXbtnBufferSize)
{
	for (auto i=0; i < xQ_.capacity(); ++i) {
		xQ_.push_back(0); // prepopulate so that averaging is simple
	}

	assert(_nh.param("throttle_axis", throttle_axis_, 1));
	assert(_nh.param("steer_axis", steer_axis_, 0));

	assert(_nh.param("throttle_gain", throttle_gain_, 1.f));
	assert(_nh.param("steer_gain", steer_gain_, 1.f));

	assert(_nh.param("deadman_btn", deadman_btn_, 4));

	// assert(_nh.param("tank_axis", tank_axis_, 3));
	assert(_nh.param("tank_btn", tank_btn_, 5));

	assert(_nh.param("x_btn", x_btn_, 8));
	// ROS_ERROR("steer_gain %f", steer_gain_);
}

void DbwNode::onJointState(const sensor_msgs::JointState::ConstPtr &state) {
    // ROS_INFO("DbwNode %zd", state->name.size());
    for (auto i = 0; i < state->name.size(); ++i) {
        if (state->name.at(i) == "steering_right_front_joint")
            _dr = state->position.at(i);
        if (state->name.at(i) == "steering_left_front_joint")
            _dl = state->position.at(i);
    }
    // ROS_INFO("DbwNode steering angles %.2f %.2f", _dl, _dr);
}

void DbwNode::onInput(const sensor_msgs::Joy::ConstPtr& input) {
#if 0
	if (steer_axis_ >= input->axes.size()) {
		ROS_ERROR("Precondition violation: invalid steer_axis");
		return;
	}
	if (throttle_axis_ >= input->axes.size()) {
		ROS_ERROR("Precondition violation: invalid throttle_axis");
		return;
	}
#endif
	ROS_DEBUG("onInput %d %.2f %.2f", input->buttons[deadman_btn_]
			, input->axes[steer_axis_], input->axes[throttle_axis_]);
	geometry_msgs::Twist vel;
	if (input->buttons[deadman_btn_]) {
		if (input->buttons[tank_btn_]) {
			vel.angular.x = 1; // indicate the tank mode
		}
		vel.angular.z = steer_gain_ * input->axes[steer_axis_];
		vel.linear.x = throttle_gain_ * input->axes[throttle_axis_];
		pub_.publish(vel);
	}

	xQ_.push_back((int8_t)input->buttons[x_btn_]);
	int8_t xSum = std::accumulate(xQ_.begin(), xQ_.end(), 0);
	if (x_btn_state) {// debounce with hysterisis filter
		if (xSum < 1) {
			x_btn_state = false;
		}
	} else {
		if (xSum > (kXbtnBufferSize - 1)) {
			x_btn_state = true;
			//startPathAction();
			hcpath::parkGoal req;
			req.target = +1; // +1: dock to the front, -1: dock to the side hitches
			ac_.sendGoal(req);
		}
	}
}

#if 0
void DbwNode::startPathAction() {
	if (!ac_.isServerConnected()) {
		ROS_ERROR("hcpath server not found; cannot start path action");
		return;
	}
	try {
		auto xform = tf2_buffer_.lookupTransform("trailer", "base_footprint"
                                	, ros::Time(0));
		double x = xform.transform.rotation.x
			, y = xform.transform.rotation.y
			, z = xform.transform.rotation.z
			, w = xform.transform.rotation.w
				, yaw_est = (180.f/3.14159f)
					* atan2(2.0f * (y*z + w*x), w*w - x*x - y*y + z*z)
			;
		hcpath::parkGoal req;
		hcpath::GaussianPathState& startRef = req.start;
		hcpath::PathState& goalRef = req.goal;

		startRef.state.x = xform.transform.translation.x;
		startRef.state.y = xform.transform.translation.y;
		startRef.state.theta = yaw_est;
		//TODO: use the current steer angle
		startRef.state.kappa = 0;
		// initialize 
		ac_.sendGoal(req);
		auto ok = ac_.waitForResult(ros::Duration(.1));
		auto str = ac_.getState().toString();
		ROS_INFO("path request [%.2f, %.2f, %.2f] result %s"
				, xform.transform.translation.x, xform.transform.translation.y
				, yaw_est, str.c_str()
				);
	} catch (tf2::TransformException &ex) {
        ROS_ERROR("startPathAction failed to lookup trailer --> base_footprint xform %s"
			, ex.what());
	}
}
#endif

int main(int argc, char **argv) {
	ros::init(argc, argv, "dbw");
	DbwNode node;
	ros::spin();

	return 0;
}
