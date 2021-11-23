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
#include <hcpath/moveAction.h>
#include "hcpmpm_reeds_shepp_state_space.hpp"

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
	// ros::ServiceServer server_;
	actionlib::SimpleActionServer<hcpath::moveAction> as_;
	moveFeedback feedback_;
	moveResult result_;

	tf2_ros::Buffer buffer_;
	tf2_ros::TransformListener tf2_listener_;

	static constexpr double discretization_ = DISCRETIZATION;
	double kappa_max_, sigma_max_
		, wheel_base_, track_width_, wheel_radius_, wheel_width_;
	geometry_msgs::Point wheel_fl_pos_, wheel_fr_pos_;
	vector<geometry_msgs::Point> footprint_, wheel_;

	// filter parameters
	Motion_Noise motion_noise_;
	Measurement_Noise measurement_noise_;
	Controller controller_;

	// bool onPathRequest(hcpath::path_srv::Request &req, hcpath::path_srv::Response &res);
	void onPreempt();
	void onRequest(const moveGoalConstPtr& goal);
public:
	HcPathNode();
};

HcPathNode::HcPathNode()
: nh_("hcpath")
, ph_("~")
// , server_(nh_.advertiseService("plan", &Self::onPathRequest, this))
, as_(nh_, "hcpath", boost::bind(&Self::onRequest, this, _1)
	, false)// THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS
, tf2_listener_(buffer_)
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
    as_.start();//start() should be called after construction of the server
}

void HcPathNode::onPreempt(){
	ROS_WARN("Preempted!");
	result_.final_count = 1;
	as_.setPreempted(result_, "preempted"); 

}
void HcPathNode::onRequest(const moveGoalConstPtr &req) {
	if (!as_.isActive()) {
		ROS_WARN("Action server not active");
		return;
	}
	if (as_.isPreemptRequested()) {
		ROS_WARN("Action server preempted");
		return;
	}

	HCpmpm_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_, discretization_);
	state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
	State_With_Covariance start;
	const GaussianPathState& startRef = req->start;
	const PathState& goalRef = req->goal;
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
	goal.x = goalRef.x;
	goal.y = goalRef.y;
	goal.theta = goalRef.theta;
	goal.kappa = goalRef.kappa;
	goal.d = goalRef.d;

	ROS_INFO("path request [%.2f, %.2f, %.2f, %.2f] --> [%.2f, %.2f, %.2f, %.2f]"
		, start.state.x, start.state.y, start.state.theta, start.state.kappa
		, goal.x, goal.y, goal.theta, goal.kappa);
	vector<State> path = state_space.get_path(start.state, goal);
	ROS_WARN("Generated path length %zd", path.size());

	result_.final_count = (int32_t)path.size();
	as_.setSucceeded(result_);
	// as_.setAborted(result_, "Failed");
#if 0
	ros::Rate rate(5);
	for(progress = 1 ; progress <= goal->count; progress++){
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

#if 0
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
