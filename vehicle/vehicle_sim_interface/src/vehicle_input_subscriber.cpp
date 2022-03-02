#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <cmath>

class VehicleInputSubscriber
{
    typedef VehicleInputSubscriber Self;

    ros::NodeHandle _nh;
    ros::NodeHandle pnh_;

    ros::Publisher _rw_pub, _lw_pub //, _rw2_pub, _lw2_pub
                , _rd_pub, _ld_pub;

    ros::Subscriber twist_sub_;
    void twistCallback(const geometry_msgs::Twist::ConstPtr &input_twist_msg);

    ros::Subscriber joint_sub_;
	void onJointState(const sensor_msgs::JointState::ConstPtr &state);
    double _dr = 0, _dl = 0; // front wheel angles

#if 0
    ros::Subscriber twiststamped_sub_;
    void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr &input_twist_msg);

    ros::Subscriber steering_angle_sub_;
    void onSteeringAngle(const std_msgs::Float64::ConstPtr &input_steering_angle_msg);

    ros::Subscriber velocity_sub_;
    void velocityCallback(const std_msgs::Float64::ConstPtr &input_velocity_msg);
#endif

    double _wheel_base, _wheel_tread, _rear_tread, wheel_radius_; // params

    static constexpr double kMaxSteer = 0.59; // any more, wheel flies off to origin

  public:
    VehicleInputSubscriber();
    ~VehicleInputSubscriber(){};
};

VehicleInputSubscriber::VehicleInputSubscriber() : _nh(""), pnh_("~")
// can't figure out why this subscription doesn't fire
, _rw_pub(_nh.advertise<std_msgs::Float64>("wheel_right_front_velocity_controller/command", 1, true))
, _lw_pub(_nh.advertise<std_msgs::Float64>("wheel_left_front_velocity_controller/command", 1, true))
// , _rw2_pub(_nh.advertise<std_msgs::Float64>("wheel_right_rear_velocity_controller/command", 1, true))
// , _lw2_pub(_nh.advertise<std_msgs::Float64>("wheel_left_rear_velocity_controller/command", 1, true))
, _rd_pub(_nh.advertise<std_msgs::Float64>("steering_right_front_position_controller/command", 1, true))
, _ld_pub(_nh.advertise<std_msgs::Float64>("steering_left_front_position_controller/command", 1, true))
, twist_sub_(_nh.subscribe("cmd_vel", 1, &Self::twistCallback, this))
, joint_sub_(_nh.subscribe("/autoware_gazebo/joint_states", 1, &Self::onJointState, this))
#if 0
, twiststamped_sub_(_nh.subscribe("twist_cmd", 1, &Self::twistStampedCallback, this))
, steering_angle_sub_(_nh.subscribe("steering_angle", 1, &Self::onSteeringAngle, this))
, velocity_sub_(_nh.subscribe("velocity", 1, &Self::velocityCallback, this))
#endif
{
    _nh.param("wheel_base", _wheel_base, 0.267);
    _nh.param("wheel_radius", wheel_radius_, 0.06);
    _nh.param("wheel_tread", _wheel_tread, 0.23); // wheel_tread = 0.5 * track width
    _nh.param("rear_tread", _rear_tread, 0.15); // rear_tread = 0.5 * rear track width
}

void VehicleInputSubscriber::onJointState(const sensor_msgs::JointState::ConstPtr &state) {
    // ROS_INFO("vehicle_input_subsriber %zd", state->name.size());
    for (auto i = 0; i < state->name.size(); ++i) {
        if (state->name.at(i) == "steering_right_front_joint")
            _dr = state->position.at(i);
        if (state->name.at(i) == "steering_left_front_joint")
            _dl = state->position.at(i);
    }
    // ROS_INFO_THROTTLE(1, "vehicle_input_subsriber steering angles %.2f %.2f", _dl, _dr);
}

void VehicleInputSubscriber::twistCallback(const geometry_msgs::Twist::ConstPtr &input_twist_msg)
{
    std_msgs::Float64 r_speed, l_speed, r2_speed, l2_speed, r_delta, l_delta;

    double vref = input_twist_msg->linear.x;
    auto steer = input_twist_msg->angular.z;
    bool tankMode = input_twist_msg->angular.x != 0;
    if (tankMode) {
        l_delta.data = -kMaxSteer;
        r_delta.data =  kMaxSteer;
        auto el = -kMaxSteer - _dl, er = kMaxSteer - _dr;
        static constexpr double kEpsilonSq = 0.02*0.02;
        if (el*el < kEpsilonSq && er*er < kEpsilonSq) {
            l_speed.data = l2_speed.data = -steer;
            r_speed.data = r2_speed.data =  steer;
        }
    } else {
        if (fabs(steer) > 1E-4) {
            auto R = 1.0/steer
                , l = std::atan(_wheel_base / (R - _wheel_tread))
                , r = std::atan(_wheel_base / (R + _wheel_tread));
            ROS_DEBUG("steering %.2f, %.2f, %.2f", steer, l, r);
            l_delta.data = std::clamp(l, -kMaxSteer, kMaxSteer);
            r_delta.data = std::clamp(r, -kMaxSteer, kMaxSteer);
        } else {
            r_delta.data = l_delta.data = 0;
        }

        auto cosdel = cos(steer);
        l_speed.data = vref * (1.0 - _wheel_tread * steer);
        r_speed.data = vref * (1.0 + _wheel_tread * steer);
        l2_speed.data = l_speed.data * cosdel;
        r2_speed.data = r_speed.data * cosdel;
    }
    // ROS_INFO("manual drive %.2f, %.2f", l_delta.data, l_speed.data);
    _rw_pub.publish(r_speed);
    _lw_pub.publish(l_speed);
    // _rw2_pub.publish(r2_speed);
    // _lw2_pub.publish(l2_speed);
    _rd_pub.publish(r_delta);
    _ld_pub.publish(l_delta);
}
#if 0
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

void VehicleInputSubscriber::onSteeringAngle(const std_msgs::Float64::ConstPtr &input_steering_angle_msg)
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
#endif
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vehicle_input_subscriber");

    VehicleInputSubscriber node;
    ros::spin();

    return 0;
}
