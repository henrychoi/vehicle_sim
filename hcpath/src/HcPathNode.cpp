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
#include <gazebo_msgs/LinkStates.h>
#include <actionlib/server/simple_action_server.h> 
// #include <hcpath/path_srv.h>
#include <hcpath/parkAction.h>
#include "hcpmpm_reeds_shepp_state_space.hpp"
#include "cc_dubins_state_space.hpp"
#include "cute_c2.h"
#include <boost/circular_buffer.hpp>
#include <numeric>
#include <complex>
#include <limits>

using namespace std;
using namespace hcpath;

class HcPathNode {
	typedef HcPathNode Self;
	enum class ParkingState: uint32_t { fail = 0
		, unsafe = 0x1, stopped = 0x2
		, idle = 0x10 | static_cast<uint32_t>(stopped)
		, approaching = 0x20
			, approaching_unsafe = static_cast<uint32_t>(approaching)
					| static_cast<uint32_t>(unsafe)
		, distancing = 0x40
			, distancing_unsafe = static_cast<uint32_t>(distancing)
					| static_cast<uint32_t>(unsafe)
	} _parkingState = ParkingState::idle;

	inline uint32_t parkingStateEnum(ParkingState s) { return static_cast<uint32_t>(s); }
	void orParkingState(ParkingState substate) {
		_parkingState = static_cast<ParkingState>(
			parkingStateEnum(_parkingState) | parkingStateEnum(substate));
	}
	bool inParkingState(ParkingState parent) {
		return parkingStateEnum(_parkingState) & parkingStateEnum(parent);
	}

	int32_t _pathSign = 0; // side hitch (from the back) vs the kingpin (+1)
	bool heuristic_plan();
	bool Dubins(State& start, State& goal, vector<Control>& segments, vector<State>& path);
	bool RSDubins(State& start, State& goal, vector<Control>& segments, vector<State>& path);
	bool RSpmpm(State& start, State& goal, vector<Control>& segments, vector<State>& path);

	float dist2trailer(float x, float y, float yaw, bool logCollision=false);
    ros::NodeHandle _nh, ph_;
	ros::Publisher _planned_pub, _path_pub, _rw_pub, _lw_pub, _rd_pub, _ld_pub;

	ros::Subscriber tf_strobe_sub_;
	ros::Duration _tfWatchdogPeriod;
	ros::Time _tfTimeout = ros::Time::now(); // when the strobe watchdog expires
	void onTfStrobe(const std_msgs::Header::ConstPtr&);

    ros::Subscriber joint_sub_;
    static constexpr double kMaxThrottle = 1.5, kMaxSteer = 0.6; // 30 deg
	void onJointState(const sensor_msgs::JointState::ConstPtr &state);
	void control(double& throttle, double& curvature);

	ros::Subscriber joy_sub_;
	tf2::Vector3 _trueFootprintPose;
	void onGazeboLinkStates(const gazebo_msgs::LinkStates &links);

    int _deadman_btn;
	bool _manualOverride = false;
	void onJoy(const sensor_msgs::Joy::ConstPtr& joy) {
		_manualOverride = joy->buttons[_deadman_btn];
	}

	ros::Subscriber gazebo_sub_;

	// ros::ServiceServer server_;
	actionlib::SimpleActionServer<hcpath::parkAction> as_;
	parkFeedback feedback_;
	parkResult result_;
	// bool onPathRequest(hcpath::path_srv::Request &req, hcpath::path_srv::Response &res);
	void onPreempt();
	void onGoal();
	void onRequest(const parkGoalConstPtr& goal);

	tf2_ros::Buffer tf2_buffer_;
	tf2_ros::TransformListener tf2_listener_;

	geometry_msgs::TransformStamped _xform2kingpin, _xform2hitch, _xform2fifth;
	bool _haveXform2kingpin = false, _haveXform2hitch = false, _haveXform2Fifth = false
		// Assume the transform from base_footprint to side hitch center is the same
		// as that to the fifth wheel
		;  
	double _eAxialInt = 0, _eThetaInt = 0
		, _Kforward_s, _Kforward_kappa, _Kback_axial, _Kback_intaxial // throttle gains
		, _Max_intKappa // integral limit
		, _Kback_theta, _Kback_intTheta, _Kback_kappa, _Kback_intkappa, _Kback_lateral;

	static constexpr double kPathRes = 0.05; // [m]
	double _safe_distance, _kappa_max, _sigma_max
		, _wheel_base, _track_width, _wheel_tread
		, _wheel_radius, _wheel_width;
	geometry_msgs::Point _wheel_fl_pos, _wheel_fr_pos;
	// vector<geometry_msgs::Point> _footprint, _wheel;
	c2Poly _footprintPoly, _wheelPoly, _chassisFrontPoly;// _chassisBackPoly;
	c2GJKCache _gjkCacheF[4] = { {.count=0}, {.count=0}, {.count=0}, {.count=0} }
			, _gjkCacheB[4] = { {.count=0}, {.count=0}, {.count=0}, {.count=0} };
	float _dist2Trailer = numeric_limits<float>::max();

	struct SimplePose_ { complex<tf2Scalar> heading; tf2Scalar yaw; tf2Scalar T[3]; }
		_2Dpose;

	// filter parameters
	Motion_Noise motion_noise_;
	Measurement_Noise measurement_noise_;
	Controller controller_;

	deque<Control> _openControlQ;
	deque<State> _openStateQ;
	ros::Time _waypointTimeout;
	const ros::Duration _waypointDeadline;
	double _s, _prevS = 0, _curvature;

	// deque<geometry_msgs::PoseStamped> _actual_path;
	nav_msgs::Path _actual_path;

	int8_t _gear = 0; // only -1, 0, 1 are valid

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
, gazebo_sub_(_nh.subscribe("/truth/link_states", 1, &Self::onGazeboLinkStates, this))
// , server_(_nh.advertiseService("plan", &Self::onPathRequest, this))
, _tfWatchdogPeriod(0.4)
, as_(_nh, "/path" //, boost::bind(&Self::onRequest, this, _1)
	, false)// THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS
, tf2_listener_(tf2_buffer_)
, _waypointDeadline(3)
{
	_actual_path.header.frame_id = "trailer";
	// _tfWatchdogPeriod = ros::Duration(0.4);

	_nh.param("deadman_btn", _deadman_btn, 4);

	// control gains
	_nh.param("Kforward_s", _Kforward_s, 1.);
	_nh.param("Kforward_kappa", _Kforward_kappa, 0.1);
	_nh.param("Kback_axial", _Kback_axial, 1.);
	_nh.param("Kback_intaxial", _Kback_intaxial, 0.01);

	_nh.param("Kback_theta", _Kback_theta, 3.);
	_nh.param("Kback_intTheta", _Kback_intTheta, 0.05);
	_nh.param("Kback_kappa", _Kback_kappa, 0.5);
	_nh.param("Kback_intkappa", _Kback_intkappa, 0.001);
	_nh.param("Kback_lateral", _Kback_lateral, 1.);

	assert(_nh.getParam("/path/safe_distance", _safe_distance));
	assert(_nh.getParam("/path/kappa_max", _kappa_max));
	assert(_nh.getParam("/path/sigma_max", _sigma_max));
	assert(_nh.getParam("/path/wheel_base", _wheel_base));
	assert(_nh.getParam("/path/track_width", _track_width));
	_wheel_tread = 0.5 * _track_width;
	// _max_kappa = 1.0/_track_width;
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

	// This is just a wrapper function targeting the "footprint" param
    auto footprint = costmap_2d::makeFootprintFromParams(_nh); 
	assert(footprint.size() && footprint.size() <= 8); // c2Poly limited to 8 vertices

	_footprintPoly.count = footprint.size();
	for (auto i=0; i < _footprintPoly.count; ++i) {
		_footprintPoly.verts[i].x = footprint[i].x;
		_footprintPoly.verts[i].y = footprint[i].y;
	}
	c2MakePoly(&_footprintPoly);

	_wheelPoly.count = 4;
    _wheelPoly.verts[0].x =  _wheel_radius; _wheelPoly.verts[0].y =  0.5 * _wheel_width;
    _wheelPoly.verts[1].x = -_wheel_radius; _wheelPoly.verts[1].y =  0.5 * _wheel_width;
    _wheelPoly.verts[2].x = -_wheel_radius; _wheelPoly.verts[2].y = -0.5 * _wheel_width;
    _wheelPoly.verts[3].x =  _wheel_radius; _wheelPoly.verts[3].y = -0.5 * _wheel_width;
	c2MakePoly(&_wheelPoly);

 	XmlRpc::XmlRpcValue xmlFront;
	assert(_nh.getParam("/path/chassis_front", xmlFront));
	auto chassisFront = costmap_2d::makeFootprintFromXMLRPC(xmlFront
			, "/path/chassis_front");// full param name is only for debugging
	assert(chassisFront.size());
	_chassisFrontPoly.count = chassisFront.size();
	for (auto i=0; i < _chassisFrontPoly.count; ++i) { // geometry_msgs::Point
		_chassisFrontPoly.verts[i].x = chassisFront[i].x;
		_chassisFrontPoly.verts[i].y = chassisFront[i].y;
	}
	c2MakePoly(&_chassisFrontPoly);

	as_.registerPreemptCallback(boost::bind(&Self::onPreempt, this));
	as_.registerGoalCallback(boost::bind(&Self::onGoal, this));
    as_.start();//start() should be called after construction of the server
}

float HcPathNode::dist2trailer(float x, float y, float yaw, bool logCollision) {
	static constexpr float kLegHalfBase = .27f // @see KUEparking world leg1 pose
		, kLegHalfWidth = .23f
		, kLegRadius = 0.02f;
	static constexpr c2AABB kLegAABB[4] = { // legs are modeled as tall boxes
		{ { kLegHalfBase - kLegRadius,  kLegHalfWidth - kLegRadius} // leg1
		, { kLegHalfBase + kLegRadius,  kLegHalfWidth + kLegRadius} },
		{ { kLegHalfBase - kLegRadius, -kLegHalfWidth - kLegRadius} // leg2
		, { kLegHalfBase + kLegRadius, -kLegHalfWidth + kLegRadius} },
		{ {-kLegHalfBase - kLegRadius,  kLegHalfWidth - kLegRadius} // leg3
		, {-kLegHalfBase + kLegRadius,  kLegHalfWidth + kLegRadius} },
		{ {-kLegHalfBase - kLegRadius, -kLegHalfWidth - kLegRadius} // leg4
		, {-kLegHalfBase + kLegRadius, -kLegHalfWidth + kLegRadius} }
	};
	float minDist = numeric_limits<float>::max();
	// check for collision, at (x +- 3tau_x, y +- 3tau_y, theta +- 3tau_theta):
	// 8 transforms of the nominal footprint
	// TODO: consider varying x and y
	c2x xform;
	xform.p.x = x; xform.p.y = y;
	xform.r.c = cos(yaw); xform.r.s = sin(yaw);
	for (unsigned l=0; l < sizeof(kLegAABB)/sizeof(kLegAABB[0]); ++l) {
		auto distF = c2GJK(&kLegAABB[l], C2_TYPE_AABB, NULL
						, &_chassisFrontPoly, C2_TYPE_POLY, &xform
						, NULL, NULL, true, NULL, &_gjkCacheF[l]);
		if (distF <= 0) { // collision!
			ROS_DEBUG("front collision against leg %u at yaw %.2f", l, yaw);
		} else {
			ROS_DEBUG("front distance %.2f to leg %u at yaw %.2f", distF, l, yaw);
		}
		auto distB = c2GJK(&kLegAABB[l], C2_TYPE_AABB, NULL
						, &_footprintPoly, C2_TYPE_POLY, &xform
						, NULL, NULL, true, NULL, &_gjkCacheB[l]);
		if (distB <= 0) { // collision!
			if (logCollision)
				ROS_WARN("center collision against leg %u at yaw %.2f", l, yaw);
		} else {
			ROS_DEBUG("center distance %.2f to leg %u at yaw %.2f", distB, l, yaw);
		}
		auto dist = min(distF, distB);
		minDist = min(minDist, dist);
	}
	return minDist;
}


// should fire at FPS Hz
void HcPathNode::onTfStrobe(const std_msgs::Header::ConstPtr& header) {
	const auto t0 = ros::Time::now();
	if (!_haveXform2kingpin) {
		try {
			_xform2kingpin = tf2_buffer_.lookupTransform("trailer", "kingpin"
					, ros::Time(0));
			_haveXform2kingpin = true;
		} catch (tf2::TransformException &ex) {
			ROS_ERROR("onTfStrobe trailer --> kinpin failed: %s", ex.what());
		}
	}
	if (!_haveXform2hitch) {
		try {
			_xform2hitch = tf2_buffer_.lookupTransform("trailer", "hitch_center"
					, ros::Time(0));
			_haveXform2hitch = true;
		} catch (tf2::TransformException &ex) {
			ROS_ERROR("onTfStrobe trailer --> hitch_center failed: %s", ex.what());
		}
	}
	if (!_haveXform2Fifth) {
		try {
			_xform2fifth = tf2_buffer_.lookupTransform("base_footprint", "base_5whl"
					, ros::Time(0));
			_haveXform2Fifth = true;
		} catch (tf2::TransformException &ex) {
			ROS_ERROR("onTfStrobe base_footprint -> base_5whl failed: %s"
					, ex.what());
		}
	}

	try {
		const auto xform = tf2_buffer_.lookupTransform("trailer", "base_link"
											// , t0 - _tfTimeout
											, ros::Time(0) // get the latest xform
											);
		tf2::Quaternion Q;
		tf2::fromMsg(xform.transform.rotation, Q);
		tf2::Vector3 axis = Q.getAxis();
		// Account for the axis sign
		tf2Scalar angle = pify(Q.getAngle() * (1 - 2*signbit(axis[2])));
		_2Dpose = { // assume the received pose is roughly vertical
			.heading = polar(1., angle), .yaw = angle 
			, .T = { xform.transform.translation.x
					, xform.transform.translation.y
					, xform.transform.translation.z }
		};
		ROS_DEBUG_THROTTLE(1, "trailer -> base_link = [%.2f, %.2f; %.2f, %.2f]"
				, _2Dpose.T[0], _2Dpose.T[1], axis[2], _2Dpose.yaw);
	} catch (tf2::TransformException &ex) {
		ROS_ERROR("onTfStrobe trailer --> base_link failed; reason: %s", ex.what());
		return; // all history should be valid for stat  to be valid
	}

	_tfTimeout = t0 + _tfWatchdogPeriod;

	double nominal = _pathSign > 0
			? _xform2kingpin.transform.translation.x - _xform2fifth.transform.translation.x
			: _xform2hitch.transform.translation.x + _xform2fifth.transform.translation.x
		, e_x = nominal - _2Dpose.T[0]
		, e_y = -_2Dpose.T[1]
		, e_t = pify(M_PI * signbit(_pathSign) - _2Dpose.yaw);

	if (_pathSign) { // trying to control the vehicle
		ROS_INFO_THROTTLE(1, "Driving to %d, error %.2f, %.2f, %.2f"
				, _pathSign, e_x, e_y, e_t);
		static constexpr double kEpsilon = 0.03, kEpsilonSq = kEpsilon * kEpsilon;
		if (e_x*e_x < kEpsilonSq && e_y*e_y < kEpsilonSq
		 	// loose heading requirement when docking to the kingpin
			&& (_pathSign > 0 || e_t*e_t < kEpsilonSq)) {
			ROS_ERROR("Docked successfully!");
			_parkingState = ParkingState::idle; _gear = 0;
			_pathSign = 0;
		} else if (inParkingState(ParkingState::stopped)) {
			// check if car has arrived at the goal
			// TODO: replace with contact sensor
			// try to park (again)
			ROS_ERROR("Stopped; trying again");
			if (heuristic_plan()) {
				_gear = 0; // put into N to get going
			} else { // give up
				_parkingState = ParkingState::fail;
				_gear = -128; // non-recoverable failure
			}
		}
	}
	
	if(abs(_gear) == 1) {// check for collision while driving (in gear)
		_dist2Trailer = dist2trailer(static_cast<float>(_2Dpose.T[0])
									, static_cast<float>(_2Dpose.T[1])
									, _2Dpose.yaw
									, true);
		// auto elapsed = ros::Time::now() - t0;
		// ROS_DEBUG_THROTTLE(1, "collision checking took %u ns", elapsed.nsec);
	}
}

void HcPathNode::onGoal() {
	if (!inParkingState(ParkingState::idle)) {
		ROS_ERROR("parking request in non-idle state %u", static_cast<unsigned>(_parkingState));
		return;
	}
	_pathSign = as_.acceptNewGoal()->target > 0 ? 1 : -1;// accept the new goal
	// Get the path from the current base_link pose to the dock target
	// (0: approach fifth_whl from the front or 1: side hitch from the back)
	ROS_INFO("onGoal target %d", _pathSign);
	if (heuristic_plan()) {
		_gear = 0; // put into N to get going
	} else {
		_gear = -128; // non-recoverable failure
	}
}

bool HcPathNode::Dubins(State& start, State& goal
	, vector<Control>& segments, vector<State>& path) {
	CC_Dubins_State_Space ss(_kappa_max, _sigma_max, kPathRes);
	ss.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
	segments.clear(); path.clear();
	auto len = ss.get_path(start, goal, segments, path);

	// A valid path should be less than the Manhattan distance
	auto manhattan = fabs(goal.x - start.x) + fabs(goal.y - start.y);
	if (len > 1.2 * manhattan) {
		ROS_DEBUG("Dubins dist %.2f > %.2f", len, manhattan);
		return false;
	}

	int axialDir = 1 - 2*signbit(goal.x - start.x);
	for (const auto& point: path) {			
		// ROS_INFO("waypoint d %.2f", point.d);
		if (axialDir * (point.x - start.x) >= 0) {
			continue;
		}
		// If going backward, check collision
		auto dTheta = axialDir ? point.theta : M_PI - point.theta;
		if (dist2trailer(point.x, point.y, point.theta, false)
			< _safe_distance + 0.25*fabs(dTheta)) {
			return false;
		}
	}
	return true;
}

bool HcPathNode::RSDubins(State& start, State& goal
	, vector<Control>& segments, vector<State>& path) {
	HCpmpm_Reeds_Shepp_State_Space ss(_kappa_max, _sigma_max, kPathRes);
	ss.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
	vector<Control> controls;
	vector<State> points;
	(void)ss.get_path(start, goal, controls, points);

	segments.clear(); path.clear();
	auto len = 0.0; 
	for (const auto& control: controls) {
		if (control.delta_s * start.d <= 0) continue;
		segments.push_back(control);
		len += fabs(control.delta_s);
	}
	for (const auto& point: points) {
		if (point.d * start.d <= 0) continue;
		// ROS_INFO("waypoint d %.2f", point.d);
		path.push_back(point);
	}

	// A valid path should be less than the Manhattan distance
	auto manhattan = fabs(goal.x - start.x) + fabs(goal.y - start.y);
	if (len > 1.2 * manhattan) {
		ROS_DEBUG("RSDubins dist %.2f > %.2f", len, manhattan);
		return false;
	}

	int axialDir = 1 - 2*signbit(goal.x - start.x);
	for (const auto& point: path) {			
		// ROS_INFO("waypoint d %.2f", point.d);
		if (axialDir * (point.x - start.x) >= 0) {
			continue;
		}
		auto dTheta = axialDir ? point.theta : M_PI - point.theta;
		if (dist2trailer(point.x, point.y, point.theta, false)
			< _safe_distance + 0.25*fabs(dTheta)) {
			return false;
		}
	}
	return true;
}

bool HcPathNode::RSpmpm(State& start, State& goal
	, vector<Control>& segments, vector<State>& path) {
	HCpmpm_Reeds_Shepp_State_Space ss(_kappa_max, _sigma_max, kPathRes);
	ss.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
	segments.clear(); path.clear();
	(void)ss.get_path(start, goal, segments, path);
	int axialDir = 1 - 2*signbit(goal.x - start.x);
	for (const auto& point: path) {			
		if (axialDir * (point.x - start.x) >= 0) {
			continue;
		}
		// If going backward, check collision
		auto dTheta = axialDir ? point.theta : M_PI - point.theta;
		if (dist2trailer(point.x, point.y, point.theta, false)
			< _safe_distance + 0.25*fabs(dTheta)) {
			return false;
		}
	}
	return true;
}

bool HcPathNode::heuristic_plan() {
	bool ok = false;
	auto t0 = ros::Time::now();

	vector<State> path;
	vector<Control> segments;

	auto lateralDir = 1 - 2*signbit(_2Dpose.T[1]); // am I on the L(+) or R(-) side of trailer?
	State start = { // path planner pose is always relative to the trailer
		.x = _2Dpose.T[0], .y = _2Dpose.T[1], .theta = _2Dpose.yaw
		, .kappa = _curvature // the current curvature
		, .d = -1 // backing up by default
	};
	auto dockingPt = _pathSign > 0
		? _xform2kingpin.transform.translation.x - _xform2fifth.transform.translation.x
		: _xform2hitch.transform.translation.x   + _xform2fifth.transform.translation.x;
	State goal = { .x = dockingPt, .y = 0, 
		.theta = M_PI * signbit(_pathSign), // 0 or pi
		.kappa = 0, .d = -1 // this demo just backs up to both kingpin and hitch
	};

	if (!inParkingState(ParkingState::approaching)) {// try approach
		ROS_WARN("Trying to generate approach path");
		if (_pathSign * (start.x - goal.x) > _wheel_base 
			&& !(ok = RSpmpm(start, goal, segments, path))) {
			ROS_WARN("No approaching RS [%.2f, %.2f, %.2f] --> [%.2f]"
					, start.x, start.y, start.theta, goal.x);
		}

		ok = RSDubins(start, goal, segments, path);
		if (!ok) {
			ROS_INFO("No RSDubins [%.2f, %.2f, %.2f, %.2f] --> [%.2f]"
					, start.x, start.y, start.theta, start.kappa, goal.x);
		}
		for (auto backoff = 0.5 * _wheel_base
			; !ok && backoff < _pathSign * (start.x - dockingPt)
			; backoff += _wheel_base) {
			goal.x = dockingPt + _pathSign * backoff;
#if 1
			ROS_INFO("Approaching RSDubins [%.2f, %.2f, %.2f, %.2f] --> [%.2f]"
					, start.x, start.y, start.theta, start.kappa, goal.x);
			ok = RSDubins(start, goal, segments, path);
			if (ok)
				break;
#endif
			ROS_WARN("Approaching RS++ [%.2f, %.2f, %.2f] --> [%.2f]"
					, start.x, start.y, start.theta, goal.x);
			ok = RSpmpm(start, goal, segments, path);
			if (ok)
				break;
		}
	}
	if (ok) {
		_parkingState = ParkingState::approaching;				
	} else { // cannot approach; distance
		start.d = goal.d = 1; // go forward when recovering
		for (auto backoff = 0.25*_wheel_base
			; !ok && backoff < 10 * _wheel_base
			; backoff += _wheel_base) {
			goal.kappa = 0;//(1 - signbit(goal.y)) * _kappa_max;
			goal.x = start.x + _pathSign * backoff;
			goal.y = 0.2 * lateralDir * backoff;
			goal.theta = //_pathSign > 0 ? 0 : M_PI;
				_pathSign > 0 ? goal.y : pify(M_PI - goal.y);
			ROS_WARN("Distancing RSDubins [%.2f, %.2f, %.2f, %.2f] --> [%.2f, %.2f, %.2f]"
					, start.x, start.y, start.theta, start.kappa
					, goal.x, goal.y, goal.theta);
			ok = Dubins(start, goal, segments, path);
		}
		if (ok) {
			// don't drive too far away
			auto i=0;
			for (const auto& state: path) {
				if (_pathSign * (state.x - start.x) > 10 * _wheel_base) {
					break;
				}
				++i;
			}
			path.resize(i); // chop off the states getting us too far away
			_parkingState = ParkingState::distancing;				
		}
	} 
	// auto elapsed = ros::Time::now() - t0;
	// ROS_WARN("Path planning took %u nsec", elapsed.nsec);
	if (ok) {
		_openControlQ.clear(); _openStateQ.clear();
		for (auto seg: segments) {
			// auto delta = atan(_wheel_base * seg.kappa);
			ROS_INFO("control segment %.2f, %.2f, %.2f", seg.delta_s, seg.kappa, seg.sigma);
			_openControlQ.push_back(seg);
		}
		_eThetaInt = _eAxialInt = 0;

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
	} else {
		ROS_ERROR("Path generation failed while in parking state 0x%X"
				, parkingStateEnum(_parkingState));
	}

	return ok;
}

void HcPathNode::onPreempt(){
	ROS_WARN("Preempted!");
	result_.final_count = 1;
	as_.setPreempted(result_, "preempted");
}

void HcPathNode::onRequest(const parkGoalConstPtr &req) {
	int32_t target = req->target;
	ROS_INFO("onRequest target %d", target);
	as_.setSucceeded(result_);
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
	const auto now = ros::Time::now();
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

	double throttle = 0, curvature = 0;

	if (now > _tfTimeout) {
		ROS_ERROR_THROTTLE(1, "No pose estimate watchdog; stopping");	
		_openControlQ.clear(); _openStateQ.clear();
		orParkingState(ParkingState::unsafe); _gear = -127; // ESTOP
	}
	switch(_gear) {
		case -128: throttle = 0; break;// fatal
		case -127: {//E-stopping
			static constexpr double kEpsilon = 0.01
				, kEpsilonSq = kEpsilon * kEpsilon;
			ROS_WARN_THROTTLE(1, "E-stopping; wheel speed %.2f %.2f", wl, wr);
			if (wl*wl < kEpsilonSq && wr*wr < kEpsilonSq) { // stopped
				++_gear; orParkingState(ParkingState::stopped);			
			}
		}	// fall through
		case -126:
			throttle = 0;
			orParkingState(ParkingState::idle);
			break;// Need a new path; tell the path planner and wait

		case -1: // reverse => approaching the trailer
			if (_dist2Trailer < _safe_distance) {
				ROS_ERROR_THROTTLE(1, "Unsafe distance %.3f @(%.2f, %.2f; %.2f)"
						, _dist2Trailer, _2Dpose.T[0], _2Dpose.T[1], _2Dpose.yaw);
				_openControlQ.clear(); _openStateQ.clear();
				orParkingState(ParkingState::unsafe); _gear = -127; // ESTOP
				throttle = 0;
			} else {
				control(throttle, curvature);
			}
			break;
		default: // forward or neutral
			control(throttle, curvature);
			break;
	} // end switch(_gear)

	// Done with bicycle model; distribute the bicycle model to the L/R wheels
	curvature = clamp(curvature, -_kappa_max, _kappa_max);

	std_msgs::Float64 r_speed, l_speed, r_delta, l_delta;
    if (fabs(curvature) > 1E-4) {
        auto R = 1.0/curvature
			// Require R > 2*wheel_tread (= track_width)
            , l = atan(_wheel_base / (R - _wheel_tread))
            , r = atan(_wheel_base / (R + _wheel_tread));
        l_delta.data = clamp(l, -kMaxSteer, kMaxSteer);
        r_delta.data = clamp(r, -kMaxSteer, kMaxSteer);
    // } else { // unnecessary since Float64 is initialized to 0 anyway
    //     r_delta.data = l_delta.data = 0;
    }

	// if (fabs(throttle) > 0.01) {
	// 	ROS_INFO("Gear %d, throttle %.2f, curvature %.2f"
	// 			, _gear, throttle, curvature);
	// }
	// Implement SW differential
	l_speed.data = throttle * (1.0 - _wheel_tread * curvature);
	r_speed.data = throttle * (1.0 + _wheel_tread * curvature);

	if (_manualOverride) {
		ROS_INFO_THROTTLE(0.5, // "dl %.3g, dr %.3g, sl %.3g, sr %.3g "
				"^[%.2f, %.2f, %.2f] vs truth (%.2f, %.2f, %.2f)"
				// , dl, dr, sl, sr
				, _2Dpose.T[0], _2Dpose.T[1], _2Dpose.yaw
				, _trueFootprintPose[0], _trueFootprintPose[1], _trueFootprintPose[2]);
	} else {
		ROS_INFO_THROTTLE(0.25,
			"Gear %d, throttle %.2f, curvature %.2f vs. actual %.2f"
				, _gear, throttle, curvature, _curvature);
		_rw_pub.publish(r_speed);
		_lw_pub.publish(l_speed);
		_rd_pub.publish(r_delta);
		_ld_pub.publish(l_delta);
	}
}

// @precondition Have vehicle pose estimate
void HcPathNode::control(double& throttle, double& curvature) {
	const auto now = ros::Time::now();

	auto ds = _s - _prevS; // displacement along the path
	double eAxial = 0, eLateral = 0, eTheta = 0, eKappa = 0;
	static constexpr double kCos30Deg = 0.866;
	static constexpr double kEpsilon = 0.01  // 1 cm ball
		, kEpsilonSq = kEpsilon * kEpsilon;
	if (_openControlQ.size()) {
		const auto& ctrl = _openControlQ.front();
#if 0
		if (_openStateQ.size()) {
			const auto& first = _openStateQ.front();
		}
#endif
		curvature = ctrl.kappa;
		if (!_gear) { // I use neutral to switch between gears 
			auto e = ctrl.kappa - _curvature;
			if (fabs(e) < 0.05) {
				_gear = 1 - 2*signbit(ctrl.delta_s);
				ROS_WARN("Kappa error %.2f switching gear to %d"
						, e, _gear);
				_prevS = _s; ds = _s - _prevS;
				_eAxialInt = 0; // reset integrated axial error
				_waypointTimeout = now + _waypointDeadline;
			}
		}
	}

	while (_gear && _openStateQ.size()) {
		const auto& first = _openStateQ.front();
		if (now > _waypointTimeout) {
			ROS_INFO("Giving up on waypoint (%.2f, %.2f, %.2f)"
					, first.x, first.y, first.theta);
			_openStateQ.pop_front();
			_waypointTimeout = now + _waypointDeadline;
			continue;
		}

		if (_openControlQ.size()) { // feed-forward control
			const auto& ctrl = _openControlQ.front();
			auto S = ctrl.delta_s, es = S - ds;
			if (// moved too far past the control endpoint
				(1 - 2*signbit(ctrl.delta_s)) * es < 0 //-kEpsilon
				|| (abs(es) < 0.1 && S * first.d < 0) // control in wrong direction
				|| ctrl.delta_s * first.d < 0// pruned waypoints beyond this control
				) { 
				ROS_WARN("Pruning control (%.2f, %.2f, %.2f), "
						"waypoint (%.0f %.2f, %.2f)"
						, ctrl.delta_s, ctrl.kappa, ctrl.sigma
						, first.d, first.x, first.y);
				_openControlQ.pop_front();

				_prevS = _s; ds = _s - _prevS;
				if (_openControlQ.size()) { // check for direction change
					const auto& ctrl = _openControlQ.front();
					if (S * ctrl.delta_s < 0) { // direction switch
						ROS_WARN("Switching path control to (%.2f, %.2f); N gear"
								, ctrl.delta_s, ctrl.kappa);
						_eAxialInt = 0; // reset integral since switching direction
						_gear = 0;
					}
				} else {
					ROS_WARN("Done with openloop control; N gear");
					_gear = 0; _parkingState = ParkingState::stopped;
					_actual_path.poses.clear();
				}
				continue;
			}

			// kappa = ctrl.kappa;
			curvature = ctrl.kappa + fabs(ds) * ctrl.sigma; // turn the wheel
			throttle = _Kforward_s * (ctrl.delta_s - ds) // feed-forward throttle
					+ _Kforward_kappa * curvature;

			if (_gear * first.d < 0) { 
				ROS_WARN("Gear in opposite of control; switching gear");
				_eAxialInt = 0; // reset integral since switching direction
				_gear = first.d;
				// _gear = 0;
				// continue;
			}
		} // end feed-forward

		auto ex1 = first.x - _2Dpose.T[0], ey1 = first.y - _2Dpose.T[1]
			, dist1sq = ex1*ex1 + ey1*ey1
			, phi_error = atan2(ey1, ex1) // [-pi, pi]
			, theta_e = pify(phi_error - _2Dpose.yaw)
			, cos1 = cos(theta_e)
			;
		if (dist1sq < kEpsilonSq) {
			ROS_INFO("^[%.2f, %.2f] reached waypoint 1 (%.2f, %.2f, %.2f); pruning 1"
					, _2Dpose.T[0], _2Dpose.T[1], first.x, first.y, first.theta);
			_openStateQ.pop_front();
			_waypointTimeout = now + _waypointDeadline;
			continue;
		}
		// At this point, have at least 1 waypoint to measure the error to
		auto dist1 = sqrt(dist1sq);
		if (first.d * dist1 * cos1 < kCos30Deg * kPathRes) {
			// waypoint NOT in front of the car
			ROS_WARN("(%.2f, %.2f) ds %.2f "
					"(%.3f m, %.2f rad); Pruning waypoint on side(%.2f, %.2f)"//", %.2f, %.0f"
					, _trueFootprintPose[0], _trueFootprintPose[1]//, _2Dpose.T[0], _2Dpose.T[1]
					, ds, first.d * dist1, theta_e
					, first.x, first.y // , first.theta, first.d
					);
			_openStateQ.pop_front();
 			_waypointTimeout = now + _waypointDeadline;
			if (_openStateQ.size()) { // not done
 			} else {
				ROS_WARN("Done with waypoints; N gear");
				_gear = 0; _parkingState = ParkingState::stopped;
				_actual_path.poses.clear();
			}
			continue;
		}

		if (_openStateQ.size() > 1) { // prune waypoints that are behind me already
			const auto& second = _openStateQ[1];
			auto ex2 = second.x - _2Dpose.T[0], ey2 = second.y - _2Dpose.T[1]
				, dist2sq = ex2*ex2 + ey2*ey2;
#if AGGRESSIVE_PRUNING
			if (dist2sq < kEpsilonSq) {
				ROS_WARN("Reached waypoint 2 (%.2f, %.2f, %.2f); pruning 2"
						, second.x, second.y, second.theta);
				_openStateQ.pop_front(); _openStateQ.pop_front();
				_waypointTimeout = now + _waypointDeadline;
				continue;
			}
#endif
			auto dot = ex1 * ex2 + ey1 * ey2;//to normalize, / sqrt(dist1sq)*sqrt(dist2sq)
			ROS_DEBUG("^[%.2f, %.2f, %.2f] (%.2g, %.2g).(%.2g, %.2g)"
					, _2Dpose.T[0], _2Dpose.T[1], _2Dpose.yaw, ex1, ey1, ex2, ey2);
			if (dot < sqrt(dist1sq * dist2sq) * kCos30Deg) {
				ROS_INFO("Prune past waypoint");
				_openStateQ.pop_front();
				_waypointTimeout = now + _waypointDeadline;
				continue;
			}
		}
		// linear (dist1 = dist1sq^0.5) feedback doesn't work well for small error?
		dist1 = sqrt(dist1sq);
		eAxial = dist1 * cos1; // signed, depending on whether front/back waypoint
		eLateral = dist1 * sin(theta_e);
		eKappa = first.kappa - _curvature;
		eTheta = pify(first.theta - _2Dpose.yaw);
		throttle += _Kback_axial * eAxial + _Kback_intaxial * _eAxialInt;
		_eAxialInt = clamp(_eAxialInt + eAxial, -kMaxThrottle, +kMaxThrottle);
		ROS_DEBUG_THROTTLE(0.25, // "(%.2f, %.2f, %.2f) "
				"gear %d, ds %.2f kappa(%.2f - %.2f), yaw(%.2f - %.2f), e_polar(%.2f, %.2f)"
				// , _trueFootprintPose[0], _trueFootprintPose[1], _trueFootprintPose[2]
				//, _2Dpose.T[0], _2Dpose.T[1]
				, _gear, ds
				, first.kappa, _curvature, first.theta, _2Dpose.yaw, dist1, theta_e
				);
		// visualize actual path
		if (_actual_path.poses.size()) {
			const auto& last = _actual_path.poses.back();
			auto dx = _2Dpose.T[0] - last.pose.position.x
				, dy = _2Dpose.T[1] - last.pose.position.y;
			if ((dx*dx + dy*dy) > kPathRes*kPathRes) {
				geometry_msgs::PoseStamped pose;
				pose.pose.position.x = _2Dpose.T[0];
				pose.pose.position.y = _2Dpose.T[1];
				pose.pose.orientation.z = sin(0.5 * _2Dpose.yaw);
				pose.pose.orientation.w = cos(0.5 * _2Dpose.yaw);
				_actual_path.poses.push_back(pose);
				_path_pub.publish(_actual_path);
			}
		} else {
			geometry_msgs::PoseStamped pose;
			pose.pose.position.x = _2Dpose.T[0];
			pose.pose.position.y = _2Dpose.T[1];
			pose.pose.orientation.z = sin(0.5 * _2Dpose.yaw);
			pose.pose.orientation.w = cos(0.5 * _2Dpose.yaw);
			_actual_path.poses.push_back(pose);
		}

		break;
	} // end throttle and steering calculation using waypoint
	auto kappa_fb = _Kback_kappa * eKappa
			+ _Kback_intkappa * _eThetaInt
			+ (1-2*signbit(_gear)) * (_Kback_theta * eTheta + _Kback_intTheta * _eThetaInt
									+ _Kback_lateral * eLateral);
	ROS_DEBUG_THROTTLE(0.25,
			"throttle %.2f, steer error %.2f,%.2f,%.2f => %.2f+%.2f"
			, throttle, eKappa, eTheta, eLateral
			, curvature, kappa_fb);
	curvature += kappa_fb;
	_eThetaInt = clamp(_eThetaInt + eTheta, -kMaxSteer, kMaxSteer);

	// I considered using the Cauchy distribution shape to limit the throttle
	// the curvature curvature error is large, but this is more intuitive
	auto maxThrottle = kMaxThrottle * min(1., 1./(kMaxSteer*kMaxSteer + fabs(eKappa)));
	throttle = clamp(throttle, -maxThrottle, maxThrottle);
}

void HcPathNode::onGazeboLinkStates(const gazebo_msgs::LinkStates &links) {
	// ROS_DEBUG("onGazeboLinkStates %zd", links.name.size());
	// if (!buffer_.canTransform("aruco", "quad_link", now)) {
	// 	return;
	// }
	size_t idx, base_idx = -1, trailer_idx = -1;
	for (idx=0; idx < links.name.size(); ++idx) {
		// ROS_INFO("link name %s", links.name[idx].c_str());
		if (links.name[idx] == "autoware_gazebo::base_footprint") {
			base_idx = idx;
		}
		if (links.name[idx] == "trailer::trailer") {//"trailer::aruco"
			trailer_idx = idx;
		}
	}
	if (base_idx < 0) {
		ROS_ERROR("base ground truth not found");
		return;
	}
	if (trailer_idx < 0) {
		ROS_ERROR("trailer ground truth not found");
		return;
	}
	auto& tp_link = links.pose[base_idx];
	auto& tp_trailer = links.pose[trailer_idx];

#if 1
	tf2::Quaternion Qlink, Qtrailer;
	tf2::fromMsg(tp_link.orientation, Qlink);
	tf2::fromMsg(tp_trailer.orientation, Qtrailer);
	// rotation FROM trailer TO the base_link
	const auto Q = Qlink * Qtrailer.inverse();
	tf2::Vector3 axis = Q.getAxis();
	_trueFootprintPose[2] = //pify(
		Q.getAngle() * (1 - 2*signbit(axis[2]))//);
		;
#endif
	_trueFootprintPose[0] = tp_link.position.x - tp_trailer.position.x;
	_trueFootprintPose[1] = tp_link.position.y - tp_trailer.position.y;
	// _trueFootprintPose[2] = tp_link.position.z - tp_trailer.position.z;
	ROS_DEBUG("trailer to base truth; "
			//"(%.2f, %.2f) - (%.2f, %.2f) = "
			"(%.2f, %.2f);"//" %.2f"
			// , tp_link.position.x, tp_link.position.y
			// , tp_trailer.position.x, tp_trailer.position.y
			, _trueFootprintPose[0], _trueFootprintPose[1]
			// , yaw_gt
			);
}