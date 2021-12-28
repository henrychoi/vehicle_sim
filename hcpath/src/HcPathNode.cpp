#include <string>
#include <deque>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
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
	ros::Publisher _planned_pub, _path_pub, _rw_pub, _lw_pub, _rd_pub, _ld_pub;
	ros::Subscriber tf_strobe_sub_;
    ros::Subscriber joint_sub_;

	ros::Subscriber joy_sub_;
    int _deadman_btn;
	bool _manualOverride = false;

	// ros::ServiceServer server_;
	actionlib::SimpleActionServer<hcpath::parkAction> as_;
	parkFeedback feedback_;
	parkResult result_;

	tf2_ros::Buffer tf2_buffer_;
	tf2_ros::TransformListener tf2_listener_;

	geometry_msgs::TransformStamped _xform2kingpin;
	bool _haveXform2kingpin = false;

	double _eAxialInt = 0, _eKappaInt = 0
		, _Kforward_s, _Kback_axial, _Kback_intaxial // throttle gains
		, _Kback_theta, _Kback_intkappa, _Kback_lateral;

	static constexpr double kPathRes = 0.05; // [m]
	double _kappa_max, _sigma_max
		, _wheel_base, _track_width, _wheel_tread, _max_kappa
		, _wheel_radius, _wheel_width;
	geometry_msgs::Point wheel_fl_pos_, wheel_fr_pos_;
	vector<geometry_msgs::Point> footprint_, wheel_;

	struct SimplePose_ { complex<tf2Scalar> heading; tf2Scalar yaw; tf2Scalar T[3]; };
	boost::circular_buffer<SimplePose_> poseQ_;
	static constexpr size_t kPoseQSize = 15; // same as the frame rate
	bool _havePoseAvg = false;
	tf2Scalar _rel_yaw_ave, _rel_yaw_var, _rel_x_ave, _rel_x_var, _rel_y_ave, _rel_y_var;

	// filter parameters
	Motion_Noise motion_noise_;
	Measurement_Noise measurement_noise_;
	Controller controller_;

	deque<Control> _openControlQ;
	deque<State> _openStateQ;
	double _s, _prevS = 0, _curvature;

	// deque<geometry_msgs::PoseStamped> _actual_path;
	nav_msgs::Path _actual_path;

	enum class Gear: uint8_t { N, F, R, P } _gear = Gear::N;

	void onTfStrobe(const std_msgs::Header::ConstPtr&);

	// bool onPathRequest(hcpath::path_srv::Request &req, hcpath::path_srv::Response &res);
	void onPreempt();
	void onGoal();
	void onRequest(const parkGoalConstPtr& goal);
	void onJointState(const sensor_msgs::JointState::ConstPtr &state);
	void onJoy(const sensor_msgs::Joy::ConstPtr& joy) {
		_manualOverride = joy->buttons[_deadman_btn];
	}
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
, _planned_pub(ph_.advertise<nav_msgs::Path>("planned_path", 1))
, _path_pub(ph_.advertise<nav_msgs::Path>("actual_path", 1))
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
, joy_sub_(_nh.subscribe<sensor_msgs::Joy>("/dbw/joy", 10, &Self::onJoy, this))
// , server_(_nh.advertiseService("plan", &Self::onPathRequest, this))
, as_(_nh, "/path" //, boost::bind(&Self::onRequest, this, _1)
	, false)// THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS
, tf2_listener_(tf2_buffer_)
, poseQ_(kPoseQSize)
{
	_actual_path.header.frame_id = "trailer";

	_nh.param("deadman_btn", _deadman_btn, 4);

	// control gains
	_nh.param("Kforward_s", _Kforward_s, 1.);
	_nh.param("Kback_axial", _Kback_axial, 1.);
	_nh.param("Kback_intaxial", _Kback_intaxial, 0.01);

	_nh.param("Kback_theta", _Kback_theta, 1.);
	_nh.param("Kback_intkappa", _Kback_intkappa, 0.001);
	_nh.param("Kback_lateral", _Kback_lateral, 1.);

	assert(_nh.getParam("/path/kappa_max", _kappa_max));
	assert(_nh.getParam("/path/sigma_max", _sigma_max));
	assert(_nh.getParam("/path/wheel_base", _wheel_base));
	assert(_nh.getParam("/path/track_width", _track_width));
	_wheel_tread = 0.5 * _track_width;
	_max_kappa = 1.0/_track_width;
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
		ROS_ERROR("onTfStrobe trailer --> base_link failed; reason: %s", ex.what());
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

		static constexpr double kPoseAveWeight = (1.0/kPoseQSize);
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
		_rel_yaw_ave = yaw_ave; _rel_x_ave = x_ave; _rel_y_ave = y_ave; 
		_rel_yaw_var = yaw_var; _rel_x_var = x_var; _rel_y_var = y_var;
		_havePoseAvg = true;
	} else {
		_havePoseAvg = false;
	}

	if (!_haveXform2kingpin) {
		try {
			_xform2kingpin = tf2_buffer_.lookupTransform("trailer", "kingpin", ros::Time(0));
			_haveXform2kingpin = true;
		} catch (tf2::TransformException &ex) {
			ROS_DEBUG("onTfStrobe trailer --> kinpin failed; reason: %s", ex.what());
		}
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
	State start = { // path planner pose is always relative to the trailer
			.x = _rel_x_ave, .y = _rel_y_ave, .theta = _rel_yaw_ave
			, .kappa = _curvature // the current curvature
			, .d = -1 // assume we are in R for the demo
		}, goal = { .x = _xform2kingpin.transform.translation.x
			, .y = _xform2kingpin.transform.translation.y
			, .theta = 0, .kappa = 0
			, .d = -1, // we can only back up into the kingpin
		};
	ROS_INFO("path request start [%.2f, %.2f, %.2f] --> [%.2f, %.2f]"
			, start.x, start.y, start.theta, goal.x, goal.y);
	vector<State> path;
	vector<Control> segments = state_space.get_path(start, goal, path);
	auto elapsed = ros::Time::now() - t0;
	// ROS_WARN("Generated control length %zd", seg.size());
	for (auto seg: segments) {
		// auto delta = atan(_wheel_base * seg.kappa);
		ROS_INFO("control segment %.2f, %.2f, %.2f", seg.delta_s, seg.kappa, seg.sigma);
		_openControlQ.push_back(seg);
	}
	_gear = Gear::N;
	_eKappaInt = _eAxialInt = 0;

	nav_msgs::Path nav_path;
	nav_path.header.frame_id = "trailer";
	for (const auto& state: path) {
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = state.x;
		pose.pose.position.y = state.y;
		pose.pose.orientation.z = sin(0.5 * state.theta);
		pose.pose.orientation.w = cos(0.5 * state.theta);
		nav_path.poses.push_back(pose);
		ROS_INFO("waypoint %.0f, %.2f, %.2f, %.2f, %.2f"
				, state.d, state.x, state.y, state.theta, state.kappa);
		_openStateQ.push_back(state);
    }
	_planned_pub.publish(nav_path);
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
	State start = { .x = _rel_x_ave, .y = _rel_y_ave
			, .theta = _rel_yaw_ave, .kappa = 0,
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

	double eAxial = 0, eLateral = 0, eHeading = 0, eKappa = 0;
	while (_havePoseAvg && _openStateQ.size()) {
		const auto& first = _openStateQ.front();
		auto ex1 = first.x - _rel_x_ave, ey1 = first.y - _rel_y_ave
			, dist1sq = ex1*ex1 + ey1*ey1
			, phi_error = atan2(ey1, ex1) // [-pi, pi]
			, theta_e = pify(phi_error - _rel_yaw_ave)
			, cos1 = cos(theta_e)
			;
		static constexpr double kCos30Deg = 0.866;
		static constexpr double kEpsilonSq = 0.01*0.01; // 1 cm ball
		if (dist1sq < kEpsilonSq) {
			ROS_WARN("Reached waypoint 1 (%.2f, %.2f, %.2f); pruning 1"
					, first.x, first.y, first.theta);
			_openStateQ.pop_front();
			continue;
		}
		// At this point, have at least 1 waypoint to measure the error to
		auto dist1 = sqrt(dist1sq);
		if (first.d * dist1 * cos1 < kCos30Deg * kPathRes) {
			// NOT roughly in the same direction
			ROS_WARN("(%.2f, %.2f) ds %.2f. Pruning "
					"(%.3f m, %.2f rad) -> %.2f, %.2f"//", %.2f, %.0f"
					, _rel_x_ave, _rel_y_ave, ds, dist1, theta_e, first.x, first.y
					// , first.theta, first.d
					);
			_openStateQ.pop_front();
			continue;
		}

		if (_openStateQ.size() > 1) { // prune irrelevant waypoint
			const auto& second = _openStateQ[1];
			auto ex2 = second.x - _rel_x_ave, ey2 = second.y - _rel_y_ave
				, dist2sq = ex2*ex2 + ey2*ey2;
			if (dist2sq < kEpsilonSq) {
				ROS_WARN("Reached waypoint 2 (%.2f, %.2f, %.2f); pruning 2"
						, second.x, second.y, second.theta);
				_openStateQ.pop_front(); _openStateQ.pop_front();
				continue;
			}

			auto dot = ex1 * ex2 + ey1 * ey2;//to normalize, / sqrt(dist1sq)*sqrt(dist2sq)
			ROS_DEBUG("pose (%.2f, %.2f, %.2f): (%.2g, %.2g).(%.2g, %.2g)"
					, _rel_x_ave, _rel_y_ave, _rel_yaw_ave, ex1, ey1, ex2, ey2);
			if (dot < sqrt(dist1sq * dist2sq) * kCos30Deg) {
				_openStateQ.pop_front();
				ROS_INFO("Prune past waypoint");
				continue;
			}
		}

		// linear (dist1 = dist1sq^0.5) feedback doesn't work well for small error?
		dist1 = sqrt(dist1sq);
		eAxial = dist1 * cos1; // signed, depending on whether front/back waypoint
		eLateral = dist1 * sin(theta_e);
		eKappa = first.kappa - _curvature;
		eHeading = pify(first.theta - _rel_yaw_ave);

		ROS_DEBUG("ds %.2f on way (%.2f, %.2f, %.2f); error %.2f, %.2f; %.2f, %.2f, %.2f, %.2f"
				, ds, first.x, first.y, first.theta
				, dist1, phi_error, eAxial, eLateral, eHeading, eKappa);

		_eKappaInt = clamp(_eKappaInt + eKappa, -2., 2.);
		break;
	}

	auto throttle = 0.
		, curvature = eKappa + _Kback_intkappa * _eKappaInt
					+ _Kback_theta * eHeading
					+ _Kback_lateral * eLateral
					;

    static constexpr double kMaxThrottle = 5, kMaxSteer = 0.6; // 30 deg
	_eAxialInt = clamp(_eAxialInt + eAxial, -kMaxThrottle, kMaxThrottle);

	std_msgs::Float64 r_speed, l_speed, r_delta, l_delta;
	static constexpr double kMinDs = 0.01;
	// double kappa = 0;
	switch (_gear) {
		case Gear::F:
		case Gear::R:
			if (_havePoseAvg) { // visualize actual path
				if (_actual_path.poses.size()) {
					const auto& last = _actual_path.poses.back();
					auto dx = _rel_x_ave - last.pose.position.x
						, dy = _rel_y_ave - last.pose.position.y;
					if ((dx*dx + dy*dy) > kPathRes*kPathRes) {
						geometry_msgs::PoseStamped pose;
						pose.pose.position.x = _rel_x_ave;
						pose.pose.position.y = _rel_y_ave;
						pose.pose.orientation.z = sin(0.5 * _rel_yaw_ave);
						pose.pose.orientation.w = cos(0.5 * _rel_yaw_ave);
						_actual_path.poses.push_back(pose);
						_path_pub.publish(_actual_path);
					}
				} else {
					geometry_msgs::PoseStamped pose;
					pose.pose.position.x = _rel_x_ave;
					pose.pose.position.y = _rel_y_ave;
					pose.pose.orientation.z = sin(0.5 * _rel_yaw_ave);
					pose.pose.orientation.w = cos(0.5 * _rel_yaw_ave);
					_actual_path.poses.push_back(pose);
				}
			}
			throttle = _Kback_axial * eAxial;

			while (_openControlQ.size()) { // feed-forward control
				const auto& ctrl = _openControlQ.front();
				if ((1 - 2*signbit(ctrl.delta_s)) * (ctrl.delta_s - ds) <= 0) {
					ROS_WARN("ds %.2f done with segment (%.2f, %.2f, %.2f). Moving to next"
							, ds, ctrl.delta_s, ctrl.kappa, ctrl.sigma);
					_openControlQ.pop_front();// move to the next waypoint
					_prevS = _s; // reset the relative path length
					continue;
				}

				// prune waypoints in opposite direction
				while (_openStateQ.size()) {
					const auto& first = _openStateQ.front();
					if (ctrl.delta_s * first.d >= 0) {
						break;
					}
					ROS_WARN("Pruning opposite dir %.0f waypoint %.2f, %.2f"
							, first.d, first.x, first.y);
					_openStateQ.pop_front();
				}
					
				// kappa = ctrl.kappa;
				curvature += ctrl.kappa + fabs(ds) * ctrl.sigma; // turn the wheel

				// > 0 for forward dir
				auto openThrottle = _Kforward_s * (ctrl.delta_s - ds);
				// TODO: add the integral axial speed control
				throttle += openThrottle;

				if ((_gear == Gear::F && ctrl.delta_s < 0)
					|| (_gear == Gear::R && ctrl.delta_s > 0)) { // have to switch gear
					ROS_WARN("Swiching openloop direction S %.2f, ds %.2f", ctrl.delta_s, ds);
					_gear = Gear::P;
					_prevS = _s; // reset path length
					_eAxialInt = 0; // reset integrated axial error
				}
				break;
			}
			if (!_openControlQ.size()) {
				_gear = Gear::N;
				ROS_WARN("Done with path");
				_actual_path.poses.clear();
			}
			ROS_INFO("gear %u, ds %.2f error %.2f = %.2f; %.2f, %.2f, %.2f = %.2f"
					, static_cast<uint8_t>(_gear), ds, eAxial, throttle
					, eLateral, eHeading, eKappa, curvature);
			break;

		case Gear::P:
			if (_openControlQ.size()) {
				const auto& ctrl = _openControlQ.front();
				// kappa = ctrl.kappa;
				curvature += ctrl.kappa;
				ROS_DEBUG("P: ctrl %.2f %.2f, %.2f", ctrl.delta_s, ctrl.kappa, _curvature);
				if (fabs(ctrl.kappa - _curvature) > 0.03) {
					break; // wheel has yet to catch up; stay in P
				}
				_prevS = _s;
				if (ctrl.delta_s > kMinDs) {
					ROS_WARN("P->F %.2f, %.2f, %.2f", ctrl.delta_s, ctrl.kappa, ctrl.sigma);
					_gear = Gear::F;
					break;
				}
				if (ctrl.delta_s < -kMinDs) {
					ROS_WARN("P->R %.2f, %.2f, %.2f", ctrl.delta_s, ctrl.kappa, ctrl.sigma);
					_gear = Gear::R;
					break;
				}
				_openControlQ.pop_front();// else deal with the next waypoint 
			}
			break;

		default: // Gear::N
			if (_openControlQ.size()) {
				const auto& ctrl = _openControlQ.front();
				_gear = Gear::P;
				ROS_WARN("Stopping; target kappa %.2f", ctrl.kappa);
			}
			break;
	}

	// Done with bicycle model; distribute the bicycle model to the L/R wheels
	curvature = clamp(curvature, -_max_kappa, _max_kappa);
    if (fabs(curvature) > 1E-4) {
		ROS_INFO("Gear %u, throttle %.2f, curvature %.2f vs. actual %.2f"
				, static_cast<uint8_t>(_gear), throttle, curvature, _curvature);
        auto R = 1.0/curvature
			// Require R > 2*wheel_tread (= track_width)
            , l = atan(_wheel_base / (R - _wheel_tread))
            , r = atan(_wheel_base / (R + _wheel_tread));
        l_delta.data = clamp(l, -kMaxSteer, kMaxSteer);
        r_delta.data = clamp(r, -kMaxSteer, kMaxSteer);
    } else {
        r_delta.data = l_delta.data = 0;
    }

	// I considered using the Cachy distribution shape to limit the throttle
	// the curvature curvature error is large, but this is more intuitive
	auto maxThrottle = kMaxThrottle * min(1., 1./(kMaxSteer*kMaxSteer + fabs(eKappa)));
	throttle = clamp(throttle, -maxThrottle, maxThrottle);

	// if (fabs(throttle) > 0.01) {
	// 	ROS_INFO("Gear %u, throttle %.2f, curvature %.2f"
	// 			, static_cast<uint8_t>(_gear), throttle, curvature);
	// }
	// Implement SW differential
	l_speed.data = throttle * (1.0 - _wheel_tread * curvature);
	r_speed.data = throttle * (1.0 + _wheel_tread * curvature);

	if (_manualOverride) {
		ROS_INFO_THROTTLE(1, "Manual movement, ds %.3g, curvature %.3g"
				, ds, curvature);
	} else {
		_rw_pub.publish(r_speed);
		_lw_pub.publish(l_speed);
		_rd_pub.publish(r_delta);
		_ld_pub.publish(l_delta);
	}
}

