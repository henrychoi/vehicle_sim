#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <hcpath/moveAction.h>
#include <boost/circular_buffer.hpp>
#include <numeric>

using namespace std;

class DbwNode {
	typedef DbwNode Self;
    ros::NodeHandle nh_;
	ros::Subscriber tf_strobe_sub_, joy_sub_;
	ros::Publisher pub_;
	tf2_ros::Buffer tf2_buffer_;
	tf2_ros::TransformListener tf2_listener_;
	actionlib::SimpleActionClient<hcpath::moveAction> ac_;

    int steer_axis_, throttle_axis_, deadman_btn_, x_btn_;
	float steer_gain_, throttle_gain_;

	static constexpr size_t kXbtnBufferSize = 10;
	boost::circular_buffer<int8_t> xQ_;
	bool x_btn_state = false;

	void onInput(const sensor_msgs::Joy::ConstPtr&);
	void onTfStrobe(const std_msgs::Empty::ConstPtr&);
	void startPathAction();
public:
	DbwNode();
};

DbwNode::DbwNode()
: nh_("dbw")
, tf_strobe_sub_(nh_.subscribe<std_msgs::Empty>("/aruco/tf_strobe", 10, &Self::onTfStrobe, this))
, joy_sub_(nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Self::onInput, this))
, pub_(nh_.advertise<geometry_msgs::Twist>("cmd_vel",1))
, tf2_listener_(tf2_buffer_)
, ac_("hcpath", false)// true causes the client to spin its own thread
, xQ_(kXbtnBufferSize)
{
	for (auto i=0; i < xQ_.capacity(); ++i) {
		xQ_.push_back(0); // prepopulate so that averaging is simple
	}
	assert(nh_.param("throttle_axis", throttle_axis_, 1));
	assert(nh_.param("steer_axis", steer_axis_, 0));
	assert(nh_.param("throttle_gain", throttle_gain_, 1.f));
	assert(nh_.param("steer_gain", steer_gain_, 1.f));

	assert(nh_.param("deadman_btn", deadman_btn_, 4));
	assert(nh_.param("x_btn", x_btn_, 8));
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
	geometry_msgs::Twist vel;
	if (input->buttons[deadman_btn_]) {
		vel.angular.z = steer_gain_ * input->axes[steer_axis_];
		vel.linear.x = throttle_gain_ * input->axes[throttle_axis_];
	}
	pub_.publish(vel);

	xQ_.push_back((int8_t)input->buttons[x_btn_]);
	int8_t xSum = std::accumulate(xQ_.begin(), xQ_.end(), 0);
	if (x_btn_state) {// debounce with hysterisis filter
		if (xSum < 3) {
			x_btn_state = false;
		}
	} else {
		// copy out the variance
		if (xSum > (kXbtnBufferSize - 3)) {
			x_btn_state = true;
			startPathAction();
		}
	}
}

void DbwNode::onTfStrobe(const std_msgs::Empty::ConstPtr&) {
	// variance of the last 2 seconds of pose estimate
	// ROS_INFO("onTfStrobe");
	ros::Time time = ros::Time::now() - ros::Duration(0.2);
	const ros::Duration kCameraPeriod(1.0/6);
	double yaw_sum = 0, yaw2_sum = 0;
	static constexpr unsigned N = 12;
	for (unsigned i=0; i < N; ++i) {
		try {
			auto xform = tf2_buffer_.lookupTransform("trailer", "base_footprint", time);
			double x = xform.transform.rotation.x
				, y = xform.transform.rotation.y
				, z = xform.transform.rotation.z
				, w = xform.transform.rotation.w
				, yaw_est = atan2(2.0f * (w * z + x * y)
						, 1.0f - 2.0f * (y * y + z * z))
				;
				yaw_sum += yaw_est;
				yaw2_sum += yaw_est * yaw_est;
		} catch (tf2::TransformException &ex) {
			ROS_ERROR("onTfStrobe %u failed to lookup trailer --> base_footprint xform %s"
				, i, ex.what());
			return; // all history should be valid for stat  to be valid
		}
		time -= kCameraPeriod;
	}
	double yaw_ave = yaw_sum / N, yaw_var = yaw2_sum/N - yaw_ave*yaw_ave;
	ROS_DEBUG("yaw ave: %.2f, var: %.2f",  yaw_ave, yaw_var);
}

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
			, yaw_est = atan2(2.0f * (w * z + x * y)
					, 1.0f - 2.0f * (y * y + z * z))
			;
		ROS_INFO("base_footprint pose in trailer frame [%.2f, %.2f, %.2f]"
				, xform.transform.translation.x, xform.transform.translation.y
				, yaw_est
				);
		hcpath::moveGoal req;
		hcpath::GaussianPathState& startRef = req.start;
		hcpath::PathState& goalRef = req.goal;

		startRef.state.x = xform.transform.translation.x;
		startRef.state.y = xform.transform.translation.y;
		startRef.state.theta = yaw_est;
		//TODO: use the current steer angle
		startRef.state.kappa = 0;
		// initialize 
		// startRef.sigma[4*0+0] = ;
	} catch (tf2::TransformException &ex) {
        ROS_ERROR("startPathAction failed to lookup trailer --> base_footprint xform %s"
			, ex.what());
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "dbw");
	DbwNode node;
	ros::spin();

	return 0;
}
