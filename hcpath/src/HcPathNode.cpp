#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

#include <actionlib/server/simple_action_server.h> 
// #include <hcpath/path_srv.h>
#include <hcpath/parkAction.h>
#include "hcpmpm_reeds_shepp_state_space.hpp"

#include <boost/circular_buffer.hpp>
#include <numeric>
#include <complex>

using namespace std;
using namespace hcpath;

#define FRAME_ID "world"
#define DISCRETIZATION 0.1               // [m]
#define VISUALIZATION_DURATION 2         // [s]
#define ANIMATE false                    // [-]
#define OPERATING_REGION_X 20.0          // [m]
#define OPERATING_REGION_Y 20.0          // [m]
#define OPERATING_REGION_THETA 2 * M_PI  // [rad]
#define random(lower, upper) (rand() * (upper - lower) / RAND_MAX + lower)

class HcPathNode {
	typedef HcPathNode Self;
    ros::NodeHandle nh_, ph_;
	ros::Publisher pub_path_;
	ros::Subscriber tf_strobe_sub_;
	// ros::ServiceServer server_;
	actionlib::SimpleActionServer<hcpath::parkAction> as_;
	parkFeedback feedback_;
	parkResult result_;

	tf2_ros::Buffer tf2_buffer_;
	tf2_ros::TransformListener tf2_listener_;

	static constexpr double discretization_ = DISCRETIZATION;
	double kappa_max_, sigma_max_
		, wheel_base_, track_width_, wheel_radius_, wheel_width_;
	geometry_msgs::Point wheel_fl_pos_, wheel_fr_pos_;
	vector<geometry_msgs::Point> footprint_, wheel_;

	struct SimplePose_ { complex<tf2Scalar> heading; tf2Scalar yaw; tf2Scalar T[3]; };
	boost::circular_buffer<SimplePose_> poseQ_;
	static constexpr size_t kPoseQSize = 16;
	static constexpr double kPoseAveWeight = (1.0/kPoseQSize);
	tf2Scalar rel_yaw_ave_, rel_yaw_var_, rel_x_ave_, rel_x_var_, rel_y_ave_, rel_y_var_;

	// filter parameters
	Motion_Noise motion_noise_;
	Measurement_Noise measurement_noise_;
	Controller controller_;

	void onTfStrobe(const std_msgs::Header::ConstPtr&);

	// bool onPathRequest(hcpath::path_srv::Request &req, hcpath::path_srv::Response &res);
	void onPreempt();
	void onGoal();
	void onRequest(const parkGoalConstPtr& goal);
public:
	HcPathNode();
};

HcPathNode::HcPathNode()
: nh_("hcpath")
, ph_("~")
, pub_path_(ph_.advertise<nav_msgs::Path>("visualization_path", 10))
, tf_strobe_sub_(nh_.subscribe<std_msgs::Header>("/aruco/tf_strobe", 10, &Self::onTfStrobe, this))
// , server_(nh_.advertiseService("plan", &Self::onPathRequest, this))
, as_(nh_, "/path" //, boost::bind(&Self::onRequest, this, _1)
	, false)// THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS
, tf2_listener_(tf2_buffer_)
, poseQ_(kPoseQSize)
{
	assert(nh_.getParam("/path/kappa_max", kappa_max_));
	assert(nh_.getParam("/path/sigma_max", sigma_max_));
	assert(nh_.getParam("/path/wheel_base", wheel_base_));
	assert(nh_.getParam("/path/track_width", track_width_));
	assert(nh_.getParam("/path/wheel_radius", wheel_radius_));
	assert(nh_.getParam("/path/wheel_width", wheel_width_));
	assert(nh_.getParam("/path/motion_noise/alpha1", motion_noise_.alpha1));
	assert(nh_.getParam("/path/motion_noise/alpha2", motion_noise_.alpha2));
	assert(nh_.getParam("/path/motion_noise/alpha3", motion_noise_.alpha3));
	assert(nh_.getParam("/path/motion_noise/alpha4", motion_noise_.alpha4));
	assert(nh_.getParam("/path/measurement_noise/std_x", measurement_noise_.std_x));
	assert(nh_.getParam("/path/measurement_noise/std_y", measurement_noise_.std_y));
	assert(nh_.getParam("/path/measurement_noise/std_theta", measurement_noise_.std_theta));
	assert(nh_.getParam("/path/controller/k1", controller_.k1));
	assert(nh_.getParam("/path/controller/k2", controller_.k2));
	assert(nh_.getParam("/path/controller/k3", controller_.k3));

    geometry_msgs::Point point1, point2, point3, point4;
    point1.x =  wheel_radius_; point1.y =  0.5 * wheel_width_;
    point2.x = -wheel_radius_; point2.y =  0.5 * wheel_width_;
    point3.x = -wheel_radius_; point3.y = -0.5 * wheel_width_;
    point4.x =  wheel_radius_; point4.y = -0.5 * wheel_width_;
    wheel_.push_back(point1);
    wheel_.push_back(point2);
    wheel_.push_back(point3);
    wheel_.push_back(point4);
    footprint_ = costmap_2d::makeFootprintFromParams(nh_); // read "footprint" param

	as_.registerPreemptCallback(boost::bind(&Self::onPreempt, this));
	as_.registerGoalCallback(boost::bind(&Self::onGoal, this));
    as_.start();//start() should be called after construction of the server
}

void HcPathNode::onTfStrobe(const std_msgs::Header::ConstPtr& header) {
	try {
		const auto xform = tf2_buffer_.lookupTransform("trailer", "base_link", ros::Time(0));
		tf2::Quaternion Q;
		tf2::fromMsg(xform.transform.rotation, Q);
		tf2::Vector3 axis = Q.getAxis();
		tf2Scalar angle = Q.getAngle() * (1 - 2*signbit(axis[2]));// Account for the axis sign
		SimplePose_ pose = { // assume the received pose is roughly vertical
			.heading = polar(1., angle), .yaw = angle 
			, .T = { xform.transform.translation.x
					, xform.transform.translation.y
					, xform.transform.translation.z}
		};
		ROS_DEBUG("trailer -> base_link = [%.2f, %.2f; %.2f, %.2f]"
				, pose.T[0], pose.T[1], axis[2], pose.yaw);
		poseQ_.push_back(pose);
	} catch (tf2::TransformException &ex) {
		ROS_ERROR("onTfStrobe failed to lookup xform %s", ex.what());
		return; // all history should be valid for stat  to be valid
	}

	if (poseQ_.full()) {
		double x_sum = 0, x2_sum = 0, y_sum = 0, y2_sum = 0;
		complex<tf2Scalar> heading_sum;
		for (auto pose: poseQ_) {
			x_sum += pose.T[0]; x2_sum += pose.T[0] * pose.T[0];
			y_sum += pose.T[1]; y2_sum += pose.T[1] * pose.T[1];
			heading_sum += pose.heading;
		}

		tf2Scalar
			yaw_ave = arg(heading_sum)//NOTE: no need to divide because of vector arithmetic
			, yaw_var = 0
			, x_ave = kPoseAveWeight * x_sum, x_var = kPoseAveWeight * x2_sum - x_ave*x_ave
			, y_ave = kPoseAveWeight * y_sum, y_var = kPoseAveWeight * y2_sum - y_ave*y_ave
			;
		for (auto pose: poseQ_) {
			auto d = pify(yaw_ave - pose.yaw);
			yaw_var += d*d;
		}
		ROS_DEBUG("yaw stat [%.1f, %.2f, %.2f+-sqrt(%.2e)]"
				, abs(heading_sum), arg(heading_sum), yaw_ave, yaw_var);
		yaw_var *= kPoseAveWeight;

		ROS_INFO("pose stat [%.2f+-%.2e, %.2f+-%.2e, %.2f+-%.2e]"
				,  x_ave, x_var,  y_ave, y_var,  yaw_ave, yaw_var);

		// store the current base_link pose (relative to the middle of the trailer)
		rel_yaw_ave_ = yaw_ave; rel_yaw_var_ = yaw_var;
		rel_x_ave_ = x_ave; rel_x_var_ = x_var;
		rel_y_ave_ = y_ave; rel_y_var_ = y_var;
	}
}

void HcPathNode::onGoal() {
	int32_t target = as_.acceptNewGoal()->target;// accept the new goal
	// Get the path from the current base_link pose to the dock target
	// (0: fifth_whl or 1: side hitch)
	ROS_INFO("onGoal target %d", target);
	auto t0 = ros::Time::now();
	HCpmpm_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_, discretization_);
	state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
	State start = { .x = rel_x_ave_, .y = rel_y_ave_
			, .theta = rel_yaw_ave_, .kappa = 0,
		}, goal = { .x = 0, .y = 0, .theta = 0, .kappa = 0, .d = 1, };
	ROS_INFO("path request start [%.2f, %.2f, %.2f] --> [%.2f, %.2f]"
		, start.x, start.y, start.theta, goal.x, goal.y);
	vector<State> path = state_space.get_path(start, goal);
	auto elapsed = ros::Time::now() - t0;
	ROS_WARN("Generated path length %zd", path.size());

	nav_msgs::Path nav_path;
	nav_path.header.frame_id = "trailer";
	for (const auto& state : path) {
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = state.x;
		pose.pose.position.y = state.y;
		pose.pose.orientation.z = sin(0.5 * state.theta);
		pose.pose.orientation.w = cos(0.5 * state.theta);
		nav_path.poses.push_back(pose);
    }
	pub_path_.publish(nav_path);
}

void HcPathNode::onPreempt(){
	ROS_WARN("Preempted!");
	result_.final_count = 1;
	as_.setPreempted(result_, "preempted");
}

void HcPathNode::onRequest(const parkGoalConstPtr &req) {
	int32_t target = req->target;
#if 1
	ROS_INFO("onRequest target %d", target);
	as_.setSucceeded(result_);
#else
	if (!as_.isActive()) {
		ROS_WARN("Action server not active");
		return;
	}
	if (as_.isPreemptRequested()) {
		ROS_WARN("Action server preempted");
		return;
	}
	// as_.acceptNewGoal();

	HCpmpm_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_, discretization_);
	state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
	State start = { .x = rel_x_ave_, .y = rel_y_ave_
			, .theta = rel_yaw_ave_, .kappa = 0,
		}, goal = { .x = 0, .y = 0, .theta = 0, .kappa = 0, .d = 1, };
	ROS_INFO("path request start [%.2f, %.2f, %.2f] --> [%.2f, %.2f]"
		, start.x, start.y, start.theta, goal.x, goal.y);
	vector<State> path = state_space.get_path(start, goal);
	ROS_WARN("Generated path length %zd", path.size());

	feedback_.current_number = (int32_t)path.size();
	result_.final_count = 0;
	as_.publishFeedback(feedback_);
	// as_.setAborted(result_, "Failed");
	ros::Rate rate(5);
	for(progress = 1 ; progress <= goal->count; progress++) {
		//Check for ros
		if(!ros::ok()){
			result.final_count = progress;
			as.setAborted(result,"I failed !");
			ROS_INFO("%s Shutting down",action_name.c_str());
			break;
		}

		if(!as.isActive() || as.isPreemptRequested()){
			return;
		}       

		if(goal->count <= progress){
			ROS_INFO("%s Succeeded at getting to goal %d", action_name.c_str(), goal->count);
			result.final_count = progress;
			as.setSucceeded(result);

		}else{
			ROS_INFO("Setting to goal %d / %d",feedback.current_number,goal->count);
			feedback.current_number = progress;
			as.publishFeedback(feedback);
		}
		rate.sleep();
	} 
#endif
}

#if 0 // server callback
bool HcPathNode::onPathRequest(hcpath::path_srv::Request &req, hcpath::path_srv::Response &res)
{
	HCpmpm_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_, discretization_);
	state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
	State_With_Covariance start;
	GaussianPathState& startRef = req.start;
	start.state.x = startRef.state.x;
	start.state.y = startRef.state.y;
	start.state.theta = startRef.state.theta;
	start.state.kappa = startRef.state.kappa;
	for (auto i=0; i < 16; ++i) {
		start.Sigma[i] = startRef.sigma[i];
		start.Lambda[i] = startRef.lambda[i];
		start.covariance[i] = startRef.covariance[i];
	}
	State goal;
	goal.x = req.goal.x;
	goal.y = req.goal.y;
	goal.theta = req.goal.theta;
	goal.kappa = req.goal.kappa;
	goal.d = req.goal.d;
	vector<State_With_Covariance> path = state_space.get_path_with_covariance(start, goal);

	return true; 
}
#endif
int main(int argc, char **argv) {
	ros::init(argc, argv, "hcpath");
	HcPathNode node;
	ros::spin();

	return 0;
}
