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

	struct SimplePose_ { tf2::Quaternion Q; float T[3]; };
	boost::circular_buffer<SimplePose_> poseQ_;
	static constexpr size_t kPoseQSize = 16;

	void onInput(const sensor_msgs::Joy::ConstPtr&);
	void onTfStrobe(const std_msgs::Header::ConstPtr&);
	void startPathAction();
public:
	DbwNode();
};

DbwNode::DbwNode()
: nh_("dbw")
, tf_strobe_sub_(nh_.subscribe<std_msgs::Header>("/aruco/tf_strobe", 10, &Self::onTfStrobe, this))
, joy_sub_(nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Self::onInput, this))
, pub_(nh_.advertise<geometry_msgs::Twist>("cmd_vel",1))
, tf2_listener_(tf2_buffer_)
, ac_("/path", false)// true causes the client to spin its own thread
, xQ_(kXbtnBufferSize)
, poseQ_(kPoseQSize)
{
	for (auto i=0; i < xQ_.capacity(); ++i) {
		xQ_.push_back(0); // prepopulate so that averaging is simple
	}
#if 0
	for (auto i=0; i < poseQ_.capacity(); ++i) {
		SimplePose_ eye = {tf2::Quaternion(), {0,0,0}};
		poseQ_.push_back(eye);
	}
#endif
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

void DbwNode::onTfStrobe(const std_msgs::Header::ConstPtr& header) {
	try {
		const auto xform = tf2_buffer_.lookupTransform("trailer", "base_link", ros::Time(0));
		SimplePose_ pose;
		tf2::fromMsg(xform.transform.rotation, pose.Q);
		pose.T[0] = xform.transform.translation.x;
		pose.T[1] = xform.transform.translation.y;
		pose.T[2] = xform.transform.translation.z;
		poseQ_.push_back(pose);
		// double x = xform.transform.rotation.x
		// 	, y = xform.transform.rotation.y
		// 	, z = xform.transform.rotation.z
		// 	, w = xform.transform.rotation.w
			// , yaw_est = (180.f/3.14159f)
			// 			* atan2(2.0f*(y*z + w*x), w*w - x*x - y*y + z*z)
			;
	} catch (tf2::TransformException &ex) {
		ROS_ERROR("onTfStrobe failed to lookup xform %s", ex.what());
		return; // all history should be valid for stat  to be valid
	}

#if 1
	if (poseQ_.size() == poseQ_.capacity()) {
		double x_sum = 0, x2_sum = 0
			, y_sum = 0, y2_sum = 0
			;
		for (auto pose: poseQ_) {
			// x = xform.transform.translation.x;
			// y = xform.transform.translation.y;
			ROS_DEBUG("[%.2f, %.2f; %.2f, %.2f, %.2f, %.2f]"
				, pose.T[0], pose.T[1]
				, pose.Q.x(), pose.Q.y(), pose.Q.z(), pose.Q.w());
		}
	}

#else
	for (unsigned i=0; i < N; ++i) {
		try {
			const ros::Duration ago((i+1)/6);
			const ros::Time time = header->stamp - ago;
			const auto xform = tf2_buffer_.lookupTransform("trailer", "base_link", time);
				// .lookupTransform("aruco", "quad_link", time);
			double x = xform.transform.rotation.x
				, y = xform.transform.rotation.y
				, z = xform.transform.rotation.z
				, w = xform.transform.rotation.w
				, yaw_est = (180.f/3.14159f)
							* atan2(2.0f*(y*z + w*x), w*w - x*x - y*y + z*z)
				;
			x = xform.transform.translation.x;
			y = xform.transform.translation.y;
			ROS_INFO("%d.%03u [%.3f, %.3f, %.3f]", time.sec, time.nsec/1000000
					, x, y, yaw_est);
				// if (yaw_est < 0) yaw_est += 360.f;
				yaw_sum += yaw_est; yaw2_sum += yaw_est * yaw_est;
				x_sum += x;         x2_sum += x * x;
				y_sum += y;         y2_sum += y * y;
		} catch (tf2::TransformException &ex) {
			ROS_ERROR("onTfStrobe %u failed to lookup xform %s"
				, i, ex.what());
			return; // all history should be valid for stat  to be valid
		}
		// time -= kCameraPeriod;
	}
	double x_ave = x_sum / N, x_var = x2_sum/N - x_ave*x_ave
		, y_ave = y_sum / N, y_var = y2_sum/N - y_ave*y_ave
		;
	ROS_INFO("pose stat [%.2f/%.2f, %.2f/%.2f, %.2f/%.2f]"
		,  x_ave, x_var,  y_ave, y_var,  yaw_ave, yaw_var);
#endif
}

void DbwNode::startPathAction() {
#if 0
	if (!ac_.isServerConnected()) {
		ROS_ERROR("hcpath server not found; cannot start path action");
		return;
	}
#endif
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
		ac_.sendGoal(req);
#if 0
		auto ok = ac_.waitForResult(ros::Duration(.1));
		auto str = ac_.getState().toString();
		ROS_INFO("path request [%.2f, %.2f, %.2f] result %s"
				, xform.transform.translation.x, xform.transform.translation.y
				, yaw_est, str.c_str()
				);
#endif
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
