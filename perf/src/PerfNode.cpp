#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <gazebo_msgs/LinkStates.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

class PerfNode {
	typedef PerfNode Self;
    ros::NodeHandle nh_;
	ros::Subscriber gazebo_sub_;

	tf2_ros::Buffer buffer_;
	tf2_ros::TransformListener tf2_listener_;

	void onGazeboLinkStates(const gazebo_msgs::LinkStates &links);

public:
	PerfNode();
};

PerfNode::PerfNode()
: nh_("perf")
, gazebo_sub_(nh_.subscribe("/truth/link_states", 1,
	&Self::onGazeboLinkStates, this))
, tf2_listener_(buffer_)
{
}

#define PI 3.14159
#define TWO_PI (2*PI)
static double pify(double alpha) {
  double v = fmod(alpha, TWO_PI);
  if (v < -PI)
    v += TWO_PI;
  else if (v > PI)
    v -= TWO_PI;
  return v;
}
void PerfNode::onGazeboLinkStates(const gazebo_msgs::LinkStates &links) {\
	static ros::Time sPrevTime = ros::Time::now();
	const auto now = ros::Time::now();
	if ((now - sPrevTime).sec <= 0) {
		return;
	}
	sPrevTime = now;
	// ROS_DEBUG("onGazeboLinkStates %zd", links.name.size());
	// if (!buffer_.canTransform("aruco", "quad_link", now)) {
	// 	return;
	// }
	// ROS_INFO("onGazeboLinkStates poses %zd", links.pose.size());
	size_t idx, base_idx = -1, trailer_idx = -1;
	for(idx=0; idx < links.name.size(); ++idx) {
		// ROS_INFO("link name %s", links.name[idx].c_str());
		if (links.name[idx] == "autoware_gazebo::base_link") {
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
	tf2::Quaternion Qlink, Qtrailer;
	tf2::fromMsg(tp_link.orientation, Qlink);
	tf2::fromMsg(tp_trailer.orientation, Qtrailer);
	// rotation FROM trailer TO the base_link
	const auto Q = Qlink * Qtrailer.inverse();
	tf2::Vector3 axis = Q.getAxis();
	double yaw_gt = Q.getAngle() * (-2*signbit(axis[2])+1);		tf2::Vector3 T2base(tp_link.position.x - tp_trailer.position.x
					, tp_link.position.y - tp_trailer.position.y
					, tp_link.position.z - tp_trailer.position.z);

	ROS_DEBUG("trailer to base truth; %.2f %.2f [%.2f, %.2f; %.2f]"
			, Qlink.w(), Qtrailer.w(), T2base[0], T2base[1], yaw_gt);
	try {
		auto xform = buffer_.lookupTransform("trailer", "base_link"
                                	, ros::Time(0));
		tf2::Quaternion Qest;
		tf2::fromMsg(xform.transform.rotation, Qest);
		tf2::Vector3 axi = Qest.getAxis();
		double yaw_est = Qest.getAngle() * (-2*signbit(axi[2])+1)
			, x = xform.transform.translation.x
			, y = xform.transform.translation.y
			, yaw_e = pify(yaw_gt - yaw_est)
			, x_e = T2base[0] - x
			, y_e = T2base[1] - y
			;
		ROS_INFO("base_ pose in trailer frame yaw [%.2f/%.2f, %.2f/%.2e, %.2f/%.2f]"
				, x, x_e, y, y_e, yaw_est, yaw_e
				);
	} catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "perf");
	PerfNode node;
	ros::spin();

	return 0;
}
