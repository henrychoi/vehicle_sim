#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetLinkState.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <cmath>

class VehicleInfoPublisher
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceClient client_;
    ros::Publisher vehicle_pose_pub_;
    ros::Publisher vehicle_vel_pub_;
    ros::Publisher steering_angle_pub_;
    ros::Timer publish_timer_; // publish timer
    ros::Subscriber odom_sub_;
    std::string ns_;
    void publishTimerCallback(const ros::TimerEvent &e);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &input_msg);
    double wheel_base_, wheel_tread_, wheel_radius_; // params

  public:
    VehicleInfoPublisher();
    ~VehicleInfoPublisher(){};
};

VehicleInfoPublisher::VehicleInfoPublisher() : nh_(""), pnh_("~")
{
    double publish_pose_rate;
    pnh_.param<double>("publish_pose_rate", publish_pose_rate, double(10.0));
    pnh_.param("ns", ns_, std::string("autoware_gazebo"));
    nh_.param("wheel_base", wheel_base_, 0.267);
    nh_.param("wheel_radius", wheel_radius_, 0.06);
    nh_.param("wheel_tread", wheel_tread_, 0.23); // wheel_tread = 0.5 * track width
 
    client_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    vehicle_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vehicle_info/pose", 1, true);
    vehicle_vel_pub_ = nh_.advertise<std_msgs::Float64>("/vehicle_info/velocity", 1, true);
    steering_angle_pub_ = nh_.advertise<std_msgs::Float64>("/vehicle_info/steering_angle", 1, true);

    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_pose_rate), &VehicleInfoPublisher::publishTimerCallback, this);
    odom_sub_ = nh_.subscribe("joint_states", 1, &VehicleInfoPublisher::jointStateCallback, this);
}

void VehicleInfoPublisher::publishTimerCallback(const ros::TimerEvent &e)
{
    gazebo_msgs::GetLinkState base_link_srv;
    base_link_srv.request.link_name = ns_ + "::base_link";
    base_link_srv.request.reference_frame = "";
    ros::Time current_time = ros::Time::now();
    client_.call(base_link_srv);

    geometry_msgs::PoseStamped output_pose;
    output_pose.header.frame_id = "base_link";
    output_pose.header.stamp = current_time;
    output_pose.pose = base_link_srv.response.link_state.pose;

    vehicle_pose_pub_.publish(output_pose);
}

void VehicleInfoPublisher::jointStateCallback(const sensor_msgs::JointState::ConstPtr &input_msg)
{
    std_msgs::Float64 output_vel, output_steering_angle;
    double r = 0, l = 0;
    double wheel_right_rear_vel = 0;
    double wheel_left_rear_vel = 0;
    for (size_t i = 0; i < input_msg->name.size(); ++i)
    {
        if (input_msg->name.at(i) == std::string("steering_right_front_joint"))
            r = input_msg->position.at(i);
        if (input_msg->name.at(i) == std::string("steering_left_front_joint"))
            l = input_msg->position.at(i);
        if (input_msg->name.at(i) == std::string("wheel_right_front_joint"))
            wheel_right_rear_vel = input_msg->velocity.at(i);
        if (input_msg->name.at(i) == std::string("wheel_left_front_joint"))
            wheel_left_rear_vel = input_msg->velocity.at(i);
    }
    double kl = 0, kr = 0;
    if (l > 1E-4) {
        auto tn = std::tan(l);
        //R = (wheel_base_ + wheel_tread_ * tn) / tn;
        kl = tn / (wheel_base_ + wheel_tread_ * tn);
    }
    if (r > 1E-4) {
        auto tn = std::tan(r);
        //R = (wheel_base_ - wheel_tread_ * tn_) / tn_;
        kr = tn / (wheel_base_ - wheel_tread_ * tn);
    }
    
    output_steering_angle.data = (kl + kr > 1E-3) ? 0.5 * kl * kr / (kl + kr) : 0;
    output_vel.data = 0.5 * wheel_radius_ * (wheel_left_rear_vel + wheel_right_rear_vel) / 2.0;

    vehicle_vel_pub_.publish(output_vel);
    steering_angle_pub_.publish(output_steering_angle);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vehicle_info_publisher");

    VehicleInfoPublisher node;
    ros::spin();

    return 0;
}
