#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <cmath>

class VehicleInputSubscriber
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher vel_right_pub_;
    ros::Publisher vel_left_pub_;
    ros::Publisher steering_right_front_pub_;
    ros::Publisher steering_left_front_pub_;
    ros::Subscriber twiststamped_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber steering_angle_sub_;
    ros::Subscriber velocity_sub_;

    void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr &input_twist_msg);
    void twistCallback(const geometry_msgs::Twist::ConstPtr &input_twist_msg);
    void sterringAngleCallback(const std_msgs::Float64::ConstPtr &input_steering_angle_msg);
    void velocityCallback(const std_msgs::Float64::ConstPtr &input_velocity_msg);

    double wheel_base_;
    double wheel_tread_;
    double wheel_radius_;

    static constexpr double kMaxSteer = 0.7; // 40 deg

  public:
    VehicleInputSubscriber();
    ~VehicleInputSubscriber(){};
};

VehicleInputSubscriber::VehicleInputSubscriber() : nh_(""), pnh_("~")
{

    nh_.param("wheel_base", wheel_base_, 2.95);
    nh_.param("wheel_radius", wheel_radius_, 0.341);
    nh_.param("wheel_tread", wheel_tread_, 1.55);
    vel_right_pub_ = nh_.advertise<std_msgs::Float64>("wheel_right_front_velocity_controller/command", 1, true);
    vel_left_pub_ = nh_.advertise<std_msgs::Float64>("wheel_left_front_velocity_controller/command", 1, true);
    steering_right_front_pub_ = nh_.advertise<std_msgs::Float64>("steering_right_front_position_controller/command", 1, true);
    steering_left_front_pub_ = nh_.advertise<std_msgs::Float64>("steering_left_front_position_controller/command", 1, true);
    twiststamped_sub_ = nh_.subscribe("twist_cmd", 1, &VehicleInputSubscriber::twistStampedCallback, this);
    twist_sub_ = nh_.subscribe("cmd_vel", 1, &VehicleInputSubscriber::twistCallback, this);
    steering_angle_sub_ = nh_.subscribe("steering_angle", 1, &VehicleInputSubscriber::sterringAngleCallback, this);
    velocity_sub_ = nh_.subscribe("velocity", 1, &VehicleInputSubscriber::velocityCallback, this);
}

void VehicleInputSubscriber::twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr &input_twist_msg)
{
    std_msgs::Float64 output_wheel_right_rear, output_wheel_left_rear, output_steering_right_front, output_steering_left_front;
    output_wheel_right_rear.data = input_twist_msg->twist.linear.x;
    output_wheel_left_rear.data = input_twist_msg->twist.linear.x;

    double vref_rear = input_twist_msg->twist.linear.x;
    double delta_ref = input_twist_msg->twist.angular.z;
        //std::atan(input_twist_msg->twist.angular.z
        //    * wheel_base_ / vref_rear);
    delta_ref = std::clamp(delta_ref, -kMaxSteer, kMaxSteer);

    // TODO: implement Ackermann steering
    output_steering_right_front.data = delta_ref;
    output_steering_left_front.data = delta_ref;

    vel_right_pub_.publish(output_wheel_right_rear);
    vel_left_pub_.publish(output_wheel_left_rear);
    steering_right_front_pub_.publish(output_steering_right_front);
    steering_left_front_pub_.publish(output_steering_left_front);
}

void VehicleInputSubscriber::twistCallback(const geometry_msgs::Twist::ConstPtr &input_twist_msg)
{
    std_msgs::Float64 output_wheel_right_rear, output_wheel_left_rear, output_steering_right_front, output_steering_left_front;
    output_wheel_right_rear.data = input_twist_msg->linear.x;
    output_wheel_left_rear.data = input_twist_msg->linear.x;

    double vref_rear = input_twist_msg->linear.x;
    double delta_ref = input_twist_msg->angular.z;
        //std::atan(input_twist_msg->angular.z * wheel_base_ / vref_rear);
    // delta_ref = 0.0 < vref_rear ? delta_ref : -delta_ref;
    delta_ref = std::clamp(delta_ref, -kMaxSteer, kMaxSteer);
    // ROS_INFO("steering %.2f", delta_ref);

    // TODO: implement Ackermann steering
    output_steering_right_front.data = delta_ref;
    output_steering_left_front.data = delta_ref;

    vel_right_pub_.publish(output_wheel_right_rear);
    vel_left_pub_.publish(output_wheel_left_rear);
    steering_right_front_pub_.publish(output_steering_right_front);
    steering_left_front_pub_.publish(output_steering_left_front);
}

void VehicleInputSubscriber::sterringAngleCallback(const std_msgs::Float64::ConstPtr &input_steering_angle_msg)
{
    std_msgs::Float64 output_steering_right_front, output_steering_left_front;

    double delta_ref = input_steering_angle_msg->data;
    delta_ref = std::clamp(delta_ref, -kMaxSteer, kMaxSteer);
    output_steering_right_front.data = delta_ref;
    output_steering_left_front.data = delta_ref;

    steering_right_front_pub_.publish(output_steering_right_front);
    steering_left_front_pub_.publish(output_steering_left_front);
}

void VehicleInputSubscriber::velocityCallback(const std_msgs::Float64::ConstPtr &input_velocity_msg)
{
    vel_right_pub_.publish(input_velocity_msg);
    vel_left_pub_.publish(input_velocity_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vehicle_input_subscriber");

    VehicleInputSubscriber node;
    ros::spin();

    return 0;
}
