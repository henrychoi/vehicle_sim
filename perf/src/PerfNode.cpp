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
, gazebo_sub_(nh_.subscribe("/gazebo/link_states", 1,
	&Self::onGazeboLinkStates, this))
, tf2_listener_(buffer_)
{
}

void PerfNode::onGazeboLinkStates(const gazebo_msgs::LinkStates &links) {
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
	size_t idx, footprint_idx = -1, trailer_idx = -1;
	for(idx=0; idx < links.name.size(); ++idx) {
		// ROS_INFO("link name %s", links.name[idx].c_str());
		if (links.name[idx] == "autoware_gazebo::base_footprint") {
			footprint_idx = idx;
		}
		if (links.name[idx] == "trailer::trailer") {//"trailer::aruco"
			trailer_idx = idx;
		}
	}
	if (footprint_idx < 0) {
		ROS_ERROR("base_footprint ground truth not found");
		return;
	}
	if (trailer_idx < 0) {
		ROS_ERROR("trailer ground truth not found");
		return;
	}
	auto& tp_footprint = links.pose[footprint_idx];
	auto& tp_trailer = links.pose[trailer_idx];
	tf2::Quaternion Qfootprint, Qtrailer;
	tf2::fromMsg(tp_footprint.orientation, Qfootprint);
	tf2::fromMsg(tp_trailer.orientation, Qtrailer);
	// rotation FROM trailer TO the base_footprint
	const auto Q2footprint = Qfootprint * Qtrailer.inverse();
	try {
		float x = Q2footprint.x(), y = Q2footprint.y()
			, z = Q2footprint.z(), w = Q2footprint.w();
		tf2::Vector3 T2footprint(tp_footprint.position.x - tp_trailer.position.x
			, tp_footprint.position.y - tp_trailer.position.y
			, tp_footprint.position.z - tp_trailer.position.z);
		float yaw_gt = atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
		ROS_DEBUG("trailer to footprint truth "
				// "Q = [%.2f, %.2f, %.2f, %.2f]
				"Yaw %.2f T = [%.3f, %.3f, %.3f]"
				, yaw_gt, T2footprint[0], T2footprint[1], T2footprint[2]);

		auto transformStamped = buffer_.lookupTransform("trailer", "base_footprint"
                                	, ros::Time(0));
		// tf2::Quaternion Qest;
		// tf2::fromMsg(transformStamped.transform.rotation, Qest);
		x = transformStamped.transform.rotation.x;
		y = transformStamped.transform.rotation.y;
		z = transformStamped.transform.rotation.z;
		w = transformStamped.transform.rotation.w;
		float yaw_est = atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z))
			, yaw_e = yaw_gt - yaw_est
			, x_e = T2footprint[0] - transformStamped.transform.translation.x
			, y_e = T2footprint[1] - transformStamped.transform.translation.y
			;
		ROS_INFO("base_footprint pose in trailer frame yaw %.2f/%.2f T = [%.2f/%.2f, %.2f/%.2f]"
				, yaw_e, yaw_est
				, x_e, transformStamped.transform.translation.x
				, y_e, transformStamped.transform.translation.y
				);
		ROS_DEBUG("base_footprint pose in trailer frame Q = [%.2f, %.2f, %.2f, %.2f] T = [%.3f, %.3f, %.3f]"
				, transformStamped.transform.rotation.x
				, transformStamped.transform.rotation.y
				, transformStamped.transform.rotation.z
				, transformStamped.transform.rotation.w
				, transformStamped.transform.translation.x
				, transformStamped.transform.translation.y
				, transformStamped.transform.translation.z);
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
