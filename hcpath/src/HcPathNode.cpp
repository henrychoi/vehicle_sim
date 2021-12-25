#include <string>
#include <deque>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
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
#define VISUALIZATION_DURATION 2         // [s]
#define ANIMATE false                    // [-]
#define OPERATING_REGION_X 20.0          // [m]
#define OPERATING_REGION_Y 20.0          // [m]
#define OPERATING_REGION_THETA 2 * M_PI  // [rad]
#define random(lower, upper) (rand() * (upper - lower) / RAND_MAX + lower)

class HcPathNode {
	typedef HcPathNode Self;
    ros::NodeHandle _nh, ph_;
	ros::Publisher _path_pub, _rw_pub, _lw_pub, _rd_pub, _ld_pub;
	ros::Subscriber tf_strobe_sub_;
    ros::Subscriber joint_sub_;
	// ros::ServiceServer server_;
	actionlib::SimpleActionServer<hcpath::parkAction> as_;
	parkFeedback feedback_;
	parkResult result_;

	tf2_ros::Buffer tf2_buffer_;
	tf2_ros::TransformListener tf2_listener_;

	float _steer_gain, _throttle_gain;

	static constexpr double kPathRes = 0.1; // [m]
	double _kappa_max, _sigma_max
		, _wheel_base, _track_width, _wheel_tread, _wheel_radius, _wheel_width;
	geometry_msgs::Point wheel_fl_pos_, wheel_fr_pos_;
	vector<geometry_msgs::Point> footprint_, wheel_;

	struct SimplePose_ { complex<tf2Scalar> heading; tf2Scalar yaw; tf2Scalar T[3]; };
	boost::circular_buffer<SimplePose_> poseQ_;
	static constexpr size_t kPoseQSize = 15;
	static constexpr double kPoseAveWeight = (1.0/kPoseQSize);
	tf2Scalar rel_yaw_ave_, rel_yaw_var_, rel_x_ave_, rel_x_var_, rel_y_ave_, rel_y_var_;

	// filter parameters
	Motion_Noise motion_noise_;
	Measurement_Noise measurement_noise_;
	Controller controller_;

	deque<Control> _openControlQ;
	deque<State> _openStateQ;
	double _s, _prevS = 0, _curvature;

	enum class Gear: uint8_t { N, F, R, P } _gear = Gear::N;

	void onTfStrobe(const std_msgs::Header::ConstPtr&);

	// bool onPathRequest(hcpath::path_srv::Request &req, hcpath::path_srv::Response &res);
	void onPreempt();
	void onGoal();
	void onRequest(const parkGoalConstPtr& goal);
	void onJointState(const sensor_msgs::JointState::ConstPtr &state);
public:
	HcPathNode();
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "hcpath");
	HcPathNode node;
	ros::spin();

	return 0;
}

HcPathNode::HcPathNode()
: _nh("hcpath")
, ph_("~")
, _path_pub(ph_.advertise<nav_msgs::Path>("visualization_path", 10))
, _rw_pub(_nh.advertise<std_msgs::Float64>("/autoware_gazebo/"
		"wheel_right_front_velocity_controller/command", 1, true))
, _lw_pub(_nh.advertise<std_msgs::Float64>("/autoware_gazebo/"
		"wheel_left_front_velocity_controller/command", 1, true))
, _rd_pub(_nh.advertise<std_msgs::Float64>("/autoware_gazebo/"
		"steering_right_front_position_controller/command", 1, true))
, _ld_pub(_nh.advertise<std_msgs::Float64>("/autoware_gazebo/"
		"steering_left_front_position_controller/command", 1, true))
, tf_strobe_sub_(_nh.subscribe<std_msgs::Header>("/aruco/tf_strobe", 10, &Self::onTfStrobe, this))
, joint_sub_(_nh.subscribe("/autoware_gazebo/joint_states", 1, &Self::onJointState, this))
// , server_(_nh.advertiseService("plan", &Self::onPathRequest, this))
, as_(_nh, "/path" //, boost::bind(&Self::onRequest, this, _1)
	, false)// THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS
, tf2_listener_(tf2_buffer_)
, poseQ_(kPoseQSize)
{
	_nh.param("throttle_gain", _throttle_gain, 1.f);
	_nh.param("steer_gain", _steer_gain, 1.f);

	assert(_nh.getParam("/path/kappa_max", _kappa_max));
	assert(_nh.getParam("/path/sigma_max", _sigma_max));
	assert(_nh.getParam("/path/wheel_base", _wheel_base));
	assert(_nh.getParam("/path/track_width", _track_width));
	_wheel_tread = 0.5 * _track_width;
	assert(_nh.getParam("/path/wheel_radius", _wheel_radius));
	assert(_nh.getParam("/path/wheel_width", _wheel_width));
	assert(_nh.getParam("/path/motion_noise/alpha1", motion_noise_.alpha1));
	assert(_nh.getParam("/path/motion_noise/alpha2", motion_noise_.alpha2));
	assert(_nh.getParam("/path/motion_noise/alpha3", motion_noise_.alpha3));
	assert(_nh.getParam("/path/motion_noise/alpha4", motion_noise_.alpha4));
	assert(_nh.getParam("/path/measurement_noise/std_x", measurement_noise_.std_x));
	assert(_nh.getParam("/path/measurement_noise/std_y", measurement_noise_.std_y));
	assert(_nh.getParam("/path/measurement_noise/std_theta", measurement_noise_.std_theta));
	assert(_nh.getParam("/path/controller/k1", controller_.k1));
	assert(_nh.getParam("/path/controller/k2", controller_.k2));
	assert(_nh.getParam("/path/controller/k3", controller_.k3));

    geometry_msgs::Point point1, point2, point3, point4;
    point1.x =  _wheel_radius; point1.y =  0.5 * _wheel_width;
    point2.x = -_wheel_radius; point2.y =  0.5 * _wheel_width;
    point3.x = -_wheel_radius; point3.y = -0.5 * _wheel_width;
    point4.x =  _wheel_radius; point4.y = -0.5 * _wheel_width;
    wheel_.push_back(point1);
    wheel_.push_back(point2);
    wheel_.push_back(point3);
    wheel_.push_back(point4);
    footprint_ = costmap_2d::makeFootprintFromParams(_nh); // read "footprint" param

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
		// ROS_DEBUG("yaw stat [%.1f, %.2f, %.2f+-sqrt(%.2e)]"
		// 		, abs(heading_sum), arg(heading_sum), yaw_ave, yaw_var);
		yaw_var *= kPoseAveWeight;

		ROS_DEBUG("pose stat [%.2f+-%.2e, %.2f+-%.2e, %.2f+-%.2e]"
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
	HCpmpm_Reeds_Shepp_State_Space state_space(_kappa_max, _sigma_max, kPathRes);
	state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
	State start = { .x = rel_x_ave_, .y = rel_y_ave_
			, .theta = rel_yaw_ave_, .kappa = 0, .d = -1
		}, goal = { .x = 0, .y = 0, .theta = 0, .kappa = 0, .d = -1, };
	ROS_INFO("path request start [%.2f, %.2f, %.2f] --> [%.2f, %.2f]"
		, start.x, start.y, start.theta, goal.x, goal.y);
	vector<State> path;
	vector<Control> segments = state_space.get_path(start, goal, path);
	auto elapsed = ros::Time::now() - t0;
	// ROS_WARN("Generated control length %zd", seg.size());
	for (auto seg: segments) {
		auto delta = atan(_wheel_base * seg.kappa);
		ROS_INFO("control segment %.2f, %.2f, %.2f", seg.delta_s, delta, seg.sigma);
		_openControlQ.push_back(seg);
	}
	_gear = Gear::N;

	nav_msgs::Path nav_path;
	nav_path.header.frame_id = "trailer";
	for (const auto& state : path) {
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = state.x;
		pose.pose.position.y = state.y;
		pose.pose.orientation.z = sin(0.5 * state.theta);
		pose.pose.orientation.w = cos(0.5 * state.theta);
		nav_path.poses.push_back(pose);

		_openStateQ.push_back(state);
    }
	_path_pub.publish(nav_path);
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

	HCpmpm_Reeds_Shepp_State_Space state_space(_kappa_max, _sigma_max, kPathRes);
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
	HCpmpm_Reeds_Shepp_State_Space state_space(_kappa_max, _sigma_max, kPathRes);
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


void HcPathNode::onJointState(const sensor_msgs::JointState::ConstPtr &state)
{
    double dr = 0, dl = 0, wl = 0, wr = 0, sl = 0, sr = 0;
    for (auto i = 0; i < state->name.size(); ++i) {
        if (state->name.at(i) == "steering_right_front_joint")
            dr = state->position.at(i);
        if (state->name.at(i) == "steering_left_front_joint")
            dl = state->position.at(i);
        if (state->name.at(i) == "wheel_right_front_joint") {
            wr = state->velocity.at(i);
			sr = state->position.at(i);
		}
        if (state->name.at(i) == "wheel_left_front_joint") {
            wl = state->velocity.at(i);
			sl = state->position.at(i);
		}
    }
    double kl = 0, kr = 0;
    if (abs(dl) > 1E-4) {
        auto tan = std::tan(dl);
        kl = tan / (_wheel_base + _wheel_tread * tan);
    }
    if (abs(dr) > 1E-4) {
        auto tan = std::tan(dr);
        kr = tan / (_wheel_base - _wheel_tread * tan);
    }
    
    _curvature = 0.5 * (kl + kr); // average curvature
	_s = _wheel_radius * 0.5 * (sl + sr);
	auto ds = _s - _prevS; // displacement along the path

	std_msgs::Float64 r_speed, l_speed, r_delta, l_delta;
	static constexpr double kMinDs = 0.01;
	double curvature = 0;
	switch (_gear) {
		case Gear::F:
		case Gear::R:
			if (_openControlQ.size()) {
				const auto& way = _openControlQ.front();
				// if (fabs(way.kappa - _curvature) > 0.01) {
				// 	_gear = Gear::P;
				// 	ROS_WARN("Cusp; kappa %.2f != %.2f", way.kappa, _curvature);
				// 	break;
				// }
				curvature = way.kappa + fabs(ds) * way.sigma; // turn the wheel
        		static constexpr double kMaxSpeed = 5.0;

				// > 0 for forward dir
				auto throttle = _throttle_gain * (way.delta_s - ds);
				throttle = clamp(throttle, -kMaxSpeed, kMaxSpeed);
				// Implement SW differential
				l_speed.data = throttle * (1.0 - _wheel_tread * way.kappa);
				r_speed.data = throttle * (1.0 + _wheel_tread * way.kappa);

				if (ds * way.delta_s < -0.01) { // have to switch gear
					_gear = Gear::P;
					ROS_WARN("Swiching direction %.2f <> %.2f", way.delta_s, ds);
					break;
				}
				if (fabs(way.delta_s - ds) < kMinDs) {
					_openControlQ.pop_front();// move to the next waypoint
					_prevS = _s; // reset the relative path length
					if (_openControlQ.size()) {
						const auto& way = _openControlQ.front();
						ROS_WARN("Next waypoint %.2f, %.2f, %.2f"
							, way.delta_s, way.kappa, way.sigma);
					}
				}
			} else {
				_gear = Gear::N;
				ROS_WARN("Done with path");
			}
			break;
		case Gear::P:
			if (_openControlQ.size()) {
				const auto& way = _openControlQ.front();
				curvature = way.kappa;
				ROS_DEBUG("waypoint ds %.2f %.2f, %.2f", way.delta_s, way.kappa, _curvature);
				if (fabs(way.kappa - _curvature) > 0.01) {
					break; // stay in P
				}
				_prevS = _s;
				if (way.delta_s > +2*kMinDs) {
					_gear = Gear::F;
					ROS_WARN("Foward %.2f, %.2f", way.delta_s, way.kappa);
					break;
				}
				if (way.delta_s < -2*kMinDs) {
					_gear = Gear::R;
					ROS_WARN("Reverse %.2f, %.2f", way.delta_s, way.kappa);
					break;
				}
				_openControlQ.pop_front();// else deal with the next waypoint 
			}
			break;
		default: // Gear::N
			if (_openControlQ.size()) {
				const auto& way = _openControlQ.front();
				_gear = Gear::P;
				ROS_WARN("Stopping; kappa %.2f", way.kappa);
			}
			break;
	}
    if (fabs(curvature) > 1E-4) {
        auto R = 1.0/curvature
            , l = atan(_wheel_base / (R - _wheel_tread))
            , r = atan(_wheel_base / (R + _wheel_tread));
        static constexpr double kMaxSteer = 0.6; // 30 deg
        l_delta.data  = clamp(l, -kMaxSteer, kMaxSteer);
        r_delta.data = clamp(r, -kMaxSteer, kMaxSteer);
    } else {
        r_delta.data = l_delta.data = 0;
    }

    _rw_pub.publish(r_speed);
    _lw_pub.publish(l_speed);
    _rd_pub.publish(r_delta);
    _ld_pub.publish(l_delta);


}
