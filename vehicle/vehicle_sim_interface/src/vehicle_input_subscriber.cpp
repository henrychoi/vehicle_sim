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
    ros::Publisher _rw_pub, _lw_pub, _rd_pub, _ld_pub;

    ros::Subscriber twiststamped_sub_;
    void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr &input_twist_msg);

    ros::Subscriber twist_sub_;
    void twistCallback(const geometry_msgs::Twist::ConstPtr &input_twist_msg);

    ros::Subscriber steering_angle_sub_;
    void sterringAngleCallback(const std_msgs::Float64::ConstPtr &input_steering_angle_msg);

    ros::Subscriber velocity_sub_;
    void velocityCallback(const std_msgs::Float64::ConstPtr &input_velocity_msg);

    double _wheel_base, _wheel_tread, wheel_radius_; // params

    static constexpr double kMaxSteer = 0.6; // 30 deg

  public:
    VehicleInputSubscriber();
    ~VehicleInputSubscriber(){};
};

VehicleInputSubscriber::VehicleInputSubscriber() : nh_(""), pnh_("~")
, _rw_pub(nh_.advertise<std_msgs::Float64>("wheel_right_front_velocity_controller/command", 1, true))
, _lw_pub(nh_.advertise<std_msgs::Float64>("wheel_left_front_velocity_controller/command", 1, true))
, _rd_pub(nh_.advertise<std_msgs::Float64>("steering_right_front_position_controller/command", 1, true))
, _ld_pub(nh_.advertise<std_msgs::Float64>("steering_left_front_position_controller/command", 1, true))
{
    nh_.param("wheel_base", _wheel_base, 0.267);
    nh_.param("wheel_radius", wheel_radius_, 0.06);
    nh_.param("wheel_tread", _wheel_tread, 0.23); // wheel_tread = 0.5 * track width
    twiststamped_sub_ = nh_.subscribe("twist_cmd", 1, &VehicleInputSubscriber::twistStampedCallback, this);
    twist_sub_ = nh_.subscribe("cmd_vel", 1, &VehicleInputSubscriber::twistCallback, this);
    steering_angle_sub_ = nh_.subscribe("steering_angle", 1, &VehicleInputSubscriber::sterringAngleCallback, this);
    velocity_sub_ = nh_.subscribe("velocity", 1, &VehicleInputSubscriber::velocityCallback, this);
}

void VehicleInputSubscriber::twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr &input_twist_msg)
{
    std_msgs::Float64 r_speed, l_speed, r_delta, l_delta;
    r_speed.data = input_twist_msg->twist.linear.x;
    l_speed.data = input_twist_msg->twist.linear.x;

    double vref_rear = input_twist_msg->twist.linear.x;
    auto kappa = input_twist_msg->twist.angular.z;
    if (fabs(kappa) > 1E-4) {
        auto R = 1.0/kappa
            , l = std::atan(_wheel_base / (R + _wheel_tread))
            , r = std::atan(_wheel_base / (R - _wheel_tread));
        ROS_DEBUG("steering kappa %.2f, %.2f, %.2f", kappa, l, r);
        l_delta.data  = std::clamp(l, -kMaxSteer, kMaxSteer);
        r_delta.data = std::clamp(r, -kMaxSteer, kMaxSteer);
    } else {
        r_delta.data = l_delta.data = 0;
    }
    _rw_pub.publish(r_speed);
    _lw_pub.publish(l_speed);
    _rd_pub.publish(r_delta);
    _ld_pub.publish(l_delta);
}

void VehicleInputSubscriber::twistCallback(const geometry_msgs::Twist::ConstPtr &input_twist_msg)
{
    std_msgs::Float64 r_speed, l_speed, r_delta, l_delta;
    r_speed.data = input_twist_msg->linear.x;
    l_speed.data = input_twist_msg->linear.x;

    double vref_rear = input_twist_msg->linear.x;
    auto steer = input_twist_msg->angular.z;
    if (fabs(steer) > 1E-4) {
        auto R = 1.0/steer
            , l = std::atan(_wheel_base / (R - _wheel_tread))
            , r = std::atan(_wheel_base / (R + _wheel_tread));
        ROS_DEBUG("steering %.2f, %.2f, %.2f", steer, l, r);
        l_delta.data  = std::clamp(l, -kMaxSteer, kMaxSteer);
        r_delta.data = std::clamp(r, -kMaxSteer, kMaxSteer);
    } else {
        r_delta.data = l_delta.data = 0;
    }

    _rw_pub.publish(r_speed);
    _lw_pub.publish(l_speed);
    _rd_pub.publish(r_delta);
    _ld_pub.publish(l_delta);
}

void VehicleInputSubscriber::sterringAngleCallback(const std_msgs::Float64::ConstPtr &input_steering_angle_msg)
{
    std_msgs::Float64 r_delta, l_delta;

    double delta_ref = input_steering_angle_msg->data;
    delta_ref = std::clamp(delta_ref, -kMaxSteer, kMaxSteer);
    r_delta.data = delta_ref;
    l_delta.data = delta_ref;

    _rd_pub.publish(r_delta);
    _ld_pub.publish(l_delta);
}

void VehicleInputSubscriber::velocityCallback(const std_msgs::Float64::ConstPtr &input_velocity_msg)
{
    _rw_pub.publish(input_velocity_msg);
    _lw_pub.publish(input_velocity_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vehicle_input_subscriber");

    VehicleInputSubscriber node;
    ros::spin();

    return 0;
}
