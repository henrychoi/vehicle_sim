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

#define random(lower, upper) (rand() * (upper - lower) / RAND_MAX + lower)


class HcPathNode {
	typedef HcPathNode Self;
	enum class ParkingState: uint32_t { unsafe = 0x1, stopped = 0x2
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
	bool Dubins(State& start, State& goal
		, vector<Control>& segments, vector<State>& path) {
		CC_Dubins_State_Space ss(_kappa_max, _sigma_max, kPathRes, start.d > 0);
		ss.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
		segments.clear(); path.clear();
		auto len = ss.get_path(start, goal, segments, path);
		// A valid path should be less than the Manhattan distance
		return len < fabs(goal.x - start.x) + fabs(goal.y - start.y);
	}

	bool RSpmpm(State& start, State& goal
		, vector<Control>& segments, vector<State>& path) {
		HCpmpm_Reeds_Shepp_State_Space ss(_kappa_max, _sigma_max, kPathRes);
		ss.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
		segments.clear(); path.clear();
		auto len = ss.get_path(start, goal, segments, path); (void)len;	
		int axialDir = 1 - 2*signbit(goal.x - start.x);
		for (const auto& point: path) {			
			if (axialDir * (point.x - start.x) >= 0) {
				continue;
			}
			// If going backward, check collision
			if (dist2trailer(point.x, point.y, point.theta, sqrt(_rel_yaw_var))
				< _safe_distance) {
				return false;
			}
		}
		return true;
	}

	float dist2trailer(float x, float y, float yaw, float sigma_yaw) {
		static constexpr float kLegBase = 0.56f, kLegWidth = 0.44f, kLegRadius = 0.02f;
		static constexpr c2AABB kLegAABB[4] = { // legs are modeled as tall boxes
			{ { 0.5f*kLegBase - kLegRadius,  0.5f*kLegWidth - kLegRadius} // leg1
			, { 0.5f*kLegBase + kLegRadius,  0.5f*kLegWidth + kLegRadius} },
			{ { 0.5f*kLegBase - kLegRadius, -0.5f*kLegWidth - kLegRadius} // leg2
			, { 0.5f*kLegBase + kLegRadius, -0.5f*kLegWidth + kLegRadius} },
			{ {-0.5f*kLegBase - kLegRadius,  0.5f*kLegWidth - kLegRadius} // leg3
			, {-0.5f*kLegBase + kLegRadius,  0.5f*kLegWidth + kLegRadius} },
			{ {-0.5f*kLegBase - kLegRadius, -0.5f*kLegWidth - kLegRadius} // leg4
			, {-0.5f*kLegBase + kLegRadius, -0.5f*kLegWidth + kLegRadius} }
		};
		float minDist = numeric_limits<float>::max();
		// check for collision, at (x +- 3tau_x, y +- 3tau_y, theta +- 3tau_theta):
		// 8 transforms of the nominal footprint
		// TODO: consider varying x and y
		c2x xform;
		xform.p.x = x; xform.p.y = y;
		for (auto i=-3; i <=3; i += 3) {
			auto yaw = _rel_yaw_ave + i*sigma_yaw;	
			xform.r.c = cos(yaw); xform.r.s = sin(yaw);
			for (unsigned l=0; l < sizeof(kLegAABB)/sizeof(kLegAABB[0]); ++l) {
				auto dist = c2GJK(&kLegAABB[l], C2_TYPE_AABB, NULL
								, &_footprintPoly, C2_TYPE_POLY, &xform
								, NULL, NULL, true, NULL, &_gjkCache[l]);
				if (dist <= 0) { // collision!
					ROS_DEBUG("collision against leg %u at yaw %.2f", l, yaw);
				} else {
					ROS_DEBUG("distance %.2f to leg %u at yaw %.2f", dist, l, yaw);
				}
				minDist = min(minDist, dist);
			}
		}
		return minDist;
	}

    ros::NodeHandle _nh, ph_;
	ros::Publisher _planned_pub, _path_pub, _rw_pub, _lw_pub, _rd_pub, _ld_pub;

	ros::Subscriber tf_strobe_sub_;
	ros::Duration _tfWatchdogPeriod;
	ros::Time _tfTimeout = ros::Time::now(); // when the strobe watchdog expires
	void onTfStrobe(const std_msgs::Header::ConstPtr&);

    ros::Subscriber joint_sub_;
    static constexpr double kMaxThrottle = 2, kMaxSteer = 0.6; // 30 deg
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

	geometry_msgs::TransformStamped _xform2kingpin, _xform2hitch;
	bool _haveXform2kingpin = false, _haveXform2hitch = false;

	double _eAxialInt = 0, _eKappaInt = 0
		, _Kforward_s, _Kback_axial, _Kback_intaxial // throttle gains
		, _Kback_theta, _Kback_kappa, _Kback_intkappa, _Kback_lateral;

	static constexpr double kPathRes = 0.05; // [m]
	double _safe_distance, _kappa_max, _sigma_max
		, _wheel_base, _track_width, _wheel_tread, _max_kappa
		, _wheel_radius, _wheel_width;
	geometry_msgs::Point _wheel_fl_pos, _wheel_fr_pos;
	// vector<geometry_msgs::Point> _footprint, _wheel;
	c2Poly _footprintPoly, _wheelPoly;//_chassisFrontPoly, _chassisBackPoly;
	c2GJKCache _gjkCache[4] = { {.count=0}, {.count=0}, {.count=0}, {.count=0} };
	float _dist2Trailer = numeric_limits<float>::max();


	struct SimplePose_ { complex<tf2Scalar> heading; tf2Scalar yaw; tf2Scalar T[3]; };
	boost::circular_buffer<SimplePose_> poseQ_;
	static constexpr size_t kPoseQSize = 7; // same as the frame rate
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
, as_(_nh, "/path" //, boost::bind(&Self::onRequest, this, _1)
	, false)// THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS
, tf2_listener_(tf2_buffer_)
, poseQ_(kPoseQSize)
{
	_actual_path.header.frame_id = "trailer";
	int fps;
	_nh.param("fps", fps, 15);
	assert(fps > 5);
	_tfWatchdogPeriod = ros::Duration(2.5 / fps);

	_nh.param("deadman_btn", _deadman_btn, 4);

	// control gains
	_nh.param("Kforward_s", _Kforward_s, 1.);
	_nh.param("Kback_axial", _Kback_axial, 1.);
	_nh.param("Kback_intaxial", _Kback_intaxial, 0.01);

	_nh.param("Kback_theta", _Kback_theta, 1.);
	_nh.param("Kback_kappa", _Kback_kappa, 0.5);
	_nh.param("Kback_intkappa", _Kback_intkappa, 0.001);
	_nh.param("Kback_lateral", _Kback_lateral, 1.);

	assert(_nh.getParam("/path/safe_distance", _safe_distance));
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

	// This is just a wrapper function targeting the "footprint" param
    auto footprint = costmap_2d::makeFootprintFromParams(_nh); 
	assert(footprint.size() && footprint.size() <= 8); // c2Poly limited to 8 vertices

#if 1
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
#else
    geometry_msgs::Point point1, point2, point3, point4;
    point1.x =  _wheel_radius; point1.y =  0.5 * _wheel_width;
    point2.x = -_wheel_radius; point2.y =  0.5 * _wheel_width;
    point3.x = -_wheel_radius; point3.y = -0.5 * _wheel_width;
    point4.x =  _wheel_radius; point4.y = -0.5 * _wheel_width;
    _wheel.push_back(point1);
    _wheel.push_back(point2);
    _wheel.push_back(point3);
    _wheel.push_back(point4);

 	XmlRpc::XmlRpcValue xmlFront, xmlBack;
	assert(_nh.getParam("/path/chassis_front", xmlFront));
	assert(_nh.getParam("/path/chassis_back", xmlBack));

	auto chassisFront = costmap_2d::makeFootprintFromXMLRPC(xmlFront
			, "/path/chassis_front") // full param name is only for debugging
		, chassisBack = costmap_2d::makeFootprintFromXMLRPC(xmlBack
			, "/path/chassis_back");
	assert(chassisFront.size());
	assert(chassisBack.size());
	_chassisFrontPoly.count = chassisFront.size();
	for (auto i=0; i < _chassisFrontPoly.count; ++i) { // geometry_msgs::Point
		_chassisFrontPoly.verts[i].x = chassisFront[i].x;
		_chassisFrontPoly.verts[i].y = chassisFront[i].y;
	}
	c2MakePoly(&_chassisFrontPoly);

	_chassisBackPoly.count = chassisBack.size();
	for (auto i=0; i < _chassisBackPoly.count; ++i) {
		_chassisBackPoly.verts[i].x = chassisBack[i].x;
		_chassisBackPoly.verts[i].y = chassisBack[i].y;
	}
	c2MakePoly(&_chassisBackPoly);
#endif
	as_.registerPreemptCallback(boost::bind(&Self::onPreempt, this));
	as_.registerGoalCallback(boost::bind(&Self::onGoal, this));
    as_.start();//start() should be called after construction of the server
}

// should fire at FPS Hz
void HcPathNode::onTfStrobe(const std_msgs::Header::ConstPtr& header) {
	const auto t0 = ros::Time::now();
	if (!_haveXform2kingpin) {
		try {
			_xform2kingpin = tf2_buffer_.lookupTransform("trailer", "kingpin", ros::Time(0));
			_haveXform2kingpin = true;
		} catch (tf2::TransformException &ex) {
			ROS_DEBUG("onTfStrobe trailer --> kinpin failed; reason: %s", ex.what());
		}
	}
	if (!_haveXform2hitch) {
		try {
			_xform2hitch = tf2_buffer_.lookupTransform("trailer", "hitch_center", ros::Time(0));
			_haveXform2hitch = true;
		} catch (tf2::TransformException &ex) {
			ROS_DEBUG("onTfStrobe trailer --> hitch_center failed; reason: %s", ex.what());
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
		tf2Scalar angle = Q.getAngle() * (1 - 2*signbit(axis[2]));// Account for the axis sign
		SimplePose_ pose = { // assume the received pose is roughly vertical
			.heading = polar(1., angle), .yaw = angle 
			, .T = { xform.transform.translation.x
					, xform.transform.translation.y
					, xform.transform.translation.z }
		};
		ROS_DEBUG("trailer -> base_link = [%.2f, %.2f; %.2f, %.2f]"
				, pose.T[0], pose.T[1], axis[2], pose.yaw);
		poseQ_.push_back(pose);
	} catch (tf2::TransformException &ex) {
		ROS_ERROR("onTfStrobe trailer --> base_link failed; reason: %s", ex.what());
		_havePoseAvg = false;
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
		_tfTimeout = t0 + _tfWatchdogPeriod;

		if (_pathSign && inParkingState(ParkingState::stopped)) {
			// check if car has arrived at the goal
			// TODO: replace with contact sensor
			double e_x = (_pathSign > 0
							? _xform2kingpin.transform.translation.x
							: _xform2hitch.transform.translation.x)
						- _rel_x_ave
				, e_y = _rel_y_ave
				, e_t = M_PI * signbit(_pathSign) - _rel_yaw_ave;
			ROS_WARN("Docking to %d, error %.2f, %.2f, %.2f", _pathSign, e_x, e_y, e_t);
			static constexpr double kEpsilon = 0.01, kEpsilonSq = kEpsilon * kEpsilon;
			if (e_t*e_t < max(_rel_yaw_var, kEpsilonSq)
				&& e_y*e_y < max(_rel_y_var, kEpsilonSq)) {
				ROS_ERROR("Docked successfully!");
				_parkingState = ParkingState::idle;
				_pathSign = 0;
			} else { // try to park (again)
				ROS_ERROR("Trying to dock");
				if (heuristic_plan()) {
					_gear = 0; // put into N to get going
				} else {
					_gear = -128; // non-recoverable failure
				}
			}
		} else if(abs(_gear) == 1) {// check for collision while driving (in gear)
			_dist2Trailer = dist2trailer(static_cast<float>(_rel_x_ave)
					, static_cast<float>(_rel_y_ave), _rel_yaw_ave, sqrt(_rel_yaw_var));
			auto elapsed = ros::Time::now() - t0;
			// ROS_DEBUG_THROTTLE(1, "collision checking took %u ns", elapsed.nsec);
		}
	} else {
		_havePoseAvg = false;
		return;
	}
}

void HcPathNode::onGoal() {
	if (_parkingState != ParkingState::idle) {
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

bool HcPathNode::heuristic_plan() {
	bool ok = false;
	auto t0 = ros::Time::now();

	vector<State> path;
	vector<Control> segments;

	// trailer half-width = 0.375; kingpin under the trailer, at 0.27, side hitch at -0.27
	// static constexpr double kKingpinOffset = 0.10;
	State goal = {
		.theta = M_PI * signbit(_pathSign)
		// , .kappa = 0
		, .d = -1 // this demo just backs up to both kingpin and hitch
	} , start = { // path planner pose is always relative to the trailer
		.x = _rel_x_ave, .y = _rel_y_ave, .theta = _rel_yaw_ave
		, .kappa = _curvature // the current curvature
		, .d = -1 // assume we are in R for the demo
	};

	auto lateralDir = 1 - 2*signbit(_rel_y_ave);
	static constexpr double kFallbackDistanceWheelbaseFactor = 3;
	if(inParkingState(ParkingState::approaching)) { // recover from collision risk
		start.d = goal.d = 1; // go forward when recovering
		goal.x = start.x + _pathSign * _wheel_base * kFallbackDistanceWheelbaseFactor;
		for (goal.y = 0
			; !ok && lateralDir * goal.y <= kFallbackDistanceWheelbaseFactor * _wheel_base
			; goal.y += 0.25 * lateralDir * kFallbackDistanceWheelbaseFactor * _wheel_base) {
			ok = Dubins(start, goal, segments, path);
			if (!ok) {
				ROS_WARN("Invalid path generated for lateral %.2f", goal.y);
			}
		}
		if (ok) {
			_parkingState = ParkingState::distancing;				
		}
	} else {// drive toward target
		double angleFromTarget;
		if (_pathSign > 0) { // dock to the front
			goal.x = _xform2kingpin.transform.translation.x
					+ abs(_rel_y_ave) + abs(pify(goal.theta - _rel_yaw_ave)) * _wheel_base;
			angleFromTarget = abs(atan2(start.y
									, start.x - _xform2kingpin.transform.translation.x));
		} else {
			goal.x = _xform2hitch.transform.translation.x
					- (abs(_rel_y_ave) + abs(pify(goal.theta - _rel_yaw_ave)) * _wheel_base);
			angleFromTarget = abs(atan2(start.y
									, _xform2hitch.transform.translation.x - start.x));
		}
		static constexpr double kPossibleApproachConeAngle = 0.5; // ~30 deg
		if (angleFromTarget*angleFromTarget
				< kPossibleApproachConeAngle*kPossibleApproachConeAngle) {
			ROS_INFO("Dubins path request [%.2f, %.2f, %.2f] %.2f --> [%.2f, %.2f]"
					, start.x, start.y, start.theta, angleFromTarget, goal.x, goal.y);
			ok = Dubins(start, goal, segments, path);
			if (!ok) {
				ROS_WARN("Invalid path generated");
			}
		}
		// outside single-pass cone
		for (goal.y = 0
			; !ok && lateralDir * goal.y <= kFallbackDistanceWheelbaseFactor * _wheel_base
			; goal.y += 0.25 * lateralDir * kFallbackDistanceWheelbaseFactor * _wheel_base) {
			// can't drive straight to the target drive to the recovery position instead
			// heuristic based recovery planner
			// move away from the trailer
			ROS_WARN("RS path request [%.2f, %.2f, %.2f] %.2f --> [%.2f, %.2f]"
					, start.x, start.y, start.theta, angleFromTarget, goal.x, goal.y);
			ok = RSpmpm(start, goal, segments, path);
		}

		if (ok) {
			_parkingState = ParkingState::approaching;				
		}
	} 

	auto elapsed = ros::Time::now() - t0;
	// ROS_WARN("Path planning took %u nsec", elapsed.nsec);
	if (ok) {
		for (auto seg: segments) {
			// auto delta = atan(_wheel_base * seg.kappa);
			ROS_INFO("control segment %.2f, %.2f, %.2f", seg.delta_s, seg.kappa, seg.sigma);
			_openControlQ.push_back(seg);
		}
		_eKappaInt = _eAxialInt = 0;
	} else {
		ROS_ERROR("Path generation failed; state 0x%X", parkingStateEnum(_parkingState));
	}
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

	if (!_havePoseAvg || now > _tfTimeout) {
		ROS_ERROR_THROTTLE(1, "No pose estimate watchdog; no vehicle control");	
		_gear = 0;
	}
	switch(_gear) {
		case -128: throttle = 0; break;// fatal
		case -127: {//E-stopping
			static constexpr double kEpsilon = 0.01
				, kEpsilonSq = kEpsilon * kEpsilon;
			ROS_WARN_THROTTLE(1, "E-stopping; wheel speed %.2f %.2f", wl, wr);
			if (wl*wl < kEpsilonSq && wr*wr < kEpsilonSq) { // stopped
				++_gear;
				orParkingState(ParkingState::stopped);			
			}
		}	// fall through
		case -126: throttle = 0; break;// Need a new path; tell the path planner and wait

		case -1: // reverse => approaching the trailer
			if (_dist2Trailer < _safe_distance) {
				ROS_ERROR_THROTTLE(1, "Unsafe distance %.3f; stopping", _dist2Trailer);
				_openControlQ.clear(); _openStateQ.clear();
				orParkingState(ParkingState::unsafe);				
				_gear = -127; // ESTOP
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
	curvature = clamp(curvature, -_max_kappa, _max_kappa);

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
				, _rel_x_ave, _rel_y_ave, _rel_yaw_ave
				, _trueFootprintPose[0], _trueFootprintPose[1], _trueFootprintPose[2]);
	} else {
		ROS_DEBUG_THROTTLE(0.2, "Gear %d, throttle %.2f, curvature %.2f vs. actual %.2f"
				, _gear, throttle, curvature, _curvature);
		_rw_pub.publish(r_speed);
		_lw_pub.publish(l_speed);
		_rd_pub.publish(r_delta);
		_ld_pub.publish(l_delta);
	}
}

// @precondition Have vehicle pose estimate
void HcPathNode::control(double& throttle, double& curvature) {
	auto ds = _s - _prevS; // displacement along the path
	double eAxial = 0, eLateral = 0, eHeading = 0, eKappa = 0;
	static constexpr double kCos30Deg = 0.866;
	static constexpr double kEpsilon = 0.01  // 1 cm ball
		, kEpsilonSq = kEpsilon * kEpsilon;

	if (_openControlQ.size()) {
		const auto& ctrl = _openControlQ.front();
		curvature = ctrl.kappa;

		if (!_gear) { // I use neutral to switch between gears 
			auto e = ctrl.kappa - _curvature;
			if (fabs(e) < 0.05) {
				_gear = 1 - 2*signbit(ctrl.delta_s);
				ROS_WARN("Kappa error %.2f switching gear to %d"
						, e, _gear);
				_prevS = _s; ds = _s - _prevS;
				_eAxialInt = 0; // reset integrated axial error
			}
		}
	}
	while (_gear && _openStateQ.size()) {
		const auto& first = _openStateQ.front();
		if (_openControlQ.size()) { // feed-forward control
			const auto& ctrl = _openControlQ.front();
			auto S = ctrl.delta_s;
			if (//ctrl.delta_s * first.d < 0 || // control in wrong direction
				// moved too far past the control endpoint
				(1 - 2*signbit(ctrl.delta_s)) * (ctrl.delta_s - ds) < 0 //-kEpsilon
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
					ROS_WARN("Done with path; N gear");
					_gear = 0;
					_parkingState = ParkingState::stopped;
					_actual_path.poses.clear();
				}
				continue;
			}

			// kappa = ctrl.kappa;
			curvature = ctrl.kappa + fabs(ds) * ctrl.sigma; // turn the wheel
			throttle = _Kforward_s * (ctrl.delta_s - ds); // feed-forward throttle
		} // end feed-forward

		auto ex1 = first.x - _rel_x_ave, ey1 = first.y - _rel_y_ave
			, dist1sq = ex1*ex1 + ey1*ey1
			, phi_error = atan2(ey1, ex1) // [-pi, pi]
			, theta_e = pify(phi_error - _rel_yaw_ave)
			, cos1 = cos(theta_e)
			;
		if (dist1sq < kEpsilonSq) {
			ROS_INFO("^[%.2f, %.2f] reached waypoint 1 (%.2f, %.2f, %.2f); pruning 1"
					, _rel_x_ave, _rel_y_ave, first.x, first.y, first.theta);
			_openStateQ.pop_front();
			continue;
		}
		// At this point, have at least 1 waypoint to measure the error to
		auto dist1 = sqrt(dist1sq);
		if (first.d * dist1 * cos1 < kCos30Deg * kPathRes) {
			// waypoint NOT in front of the car
			ROS_WARN("(%.2f, %.2f) ds %.2f "
					"(%.3f m, %.2f rad); Pruning way (%.2f, %.2f)"//", %.2f, %.0f"
					, _trueFootprintPose[0], _trueFootprintPose[1]//, _rel_x_ave, _rel_y_ave
					, ds, first.d * dist1, theta_e
					, first.x, first.y // , first.theta, first.d
					);
			_openStateQ.pop_front();
			continue;
		}

		if (_openStateQ.size() > 1) { // prune waypoints that are behind me already
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
			ROS_DEBUG("^[%.2f, %.2f, %.2f] (%.2g, %.2g).(%.2g, %.2g)"
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
		throttle += _Kback_axial * eAxial + _Kback_intaxial * _eKappaInt;
		_eAxialInt = clamp(_eAxialInt + eAxial, -kMaxThrottle, +kMaxThrottle);
		ROS_INFO_THROTTLE(0.25
				, "(%.2f, %.2f) gear %d, ds %.2f error %.2f = %.2f; %.2f, %.2f, %.2f = %.2f"
				, _trueFootprintPose[0], _trueFootprintPose[1]//, _rel_x_ave, _rel_y_ave
				, _gear, ds, eAxial, throttle
				, eLateral, eHeading, eKappa, curvature);
		// visualize actual path
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

		break;
	} // end throttle and steering calculation

	curvature += _Kback_kappa * eKappa + _Kback_intkappa * _eKappaInt
			+ _Kback_theta * eHeading
			+ _Kback_lateral * eLateral;
	_eKappaInt = clamp(_eKappaInt + eKappa, -2., 2.);

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
	_trueFootprintPose[2] = Q.getAngle() * (1 - 2*signbit(axis[2]));
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