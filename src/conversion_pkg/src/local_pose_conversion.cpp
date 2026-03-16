#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <cmath>

class LocalPoseConversion : public rclcpp::Node
{
public:
  LocalPoseConversion() : Node("local_pose_conversion")
  {
    // QoS profile compatible with PX4 micro-XRCE-DDS bridge
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    this->declare_parameter("tracking_ros.vehicle_local_position_topic", "/fmu/out/vehicle_local_position");
    local_pose_topic_ros2_ = this->get_parameter("tracking_ros.vehicle_local_position_topic").as_string();
    local_pose_topic_ros1_ = "mavros/local_position/pose";
    local_vel_topic_ros1_  = "mavros/local_position/velocity_local";
    local_acc_topic_ros1_  = "mavros/imu/data";

    local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      local_pose_topic_ros1_, qos,
      std::bind(&LocalPoseConversion::local_pose_callback, this, std::placeholders::_1));

    local_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      local_vel_topic_ros1_, qos,
      std::bind(&LocalPoseConversion::local_vel_callback, this, std::placeholders::_1));

    local_acc_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      local_acc_topic_ros1_, qos,
      std::bind(&LocalPoseConversion::local_acc_callback, this, std::placeholders::_1));

    local_pose_pub_ = this->create_publisher<px4_msgs::msg::VehicleLocalPosition>(local_pose_topic_ros2_, 10);
  }

private:
  void local_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    last_velocity_ = *msg;
    has_velocity_ = true;
  }

  void local_acc_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = *msg;
    has_imu_ = true;
  }

  void local_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    px4_msgs::msg::VehicleLocalPosition px4_msg;

    px4_msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
    px4_msg.timestamp_sample = px4_msg.timestamp;

    // ENU (x=East, y=North, z=Up) -> NED (x=North, y=East, z=Down)
    px4_msg.x =  static_cast<float>(msg->pose.position.y);
    px4_msg.y =  static_cast<float>(msg->pose.position.x);
    px4_msg.z = -static_cast<float>(msg->pose.position.z);

    // Extract yaw from ENU quaternion (ROS: CCW from East)
    // and convert to NED heading (CW from North): heading_NED = pi/2 - yaw_ENU
    const auto &q = msg->pose.orientation;
    double yaw_enu = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double heading = M_PI / 2.0 - yaw_enu;
    // Normalize to [-pi, pi]
    while (heading >  M_PI) heading -= 2.0 * M_PI;
    while (heading < -M_PI) heading += 2.0 * M_PI;
    px4_msg.heading = static_cast<float>(heading);

    px4_msg.xy_valid = true;
    px4_msg.z_valid  = true;

    if (has_velocity_) {
      const auto &v = last_velocity_.twist.linear;
      // ENU -> NED velocity: vx_NED = vy_ENU, vy_NED = vx_ENU, vz_NED = -vz_ENU
      px4_msg.vx = static_cast<float>( v.y);
      px4_msg.vy = static_cast<float>( v.x);
      px4_msg.vz = static_cast<float>(-v.z);
      px4_msg.v_xy_valid = true;
      px4_msg.v_z_valid  = true;
    } else {
      px4_msg.v_xy_valid = false;
      px4_msg.v_z_valid  = false;
    }

    if (has_imu_) {
      // IMU linear_acceleration is in FLU body frame and includes gravity.
      // Rotate body-frame accel to ENU world frame using the pose orientation,
      // subtract gravity, then convert ENU -> NED.
      const auto &a = last_imu_.linear_acceleration;
      const auto &qr = msg->pose.orientation;

      // Rotate vector v by quaternion q: v' = q * v * q_inv
      // Using the rotation matrix derived from quaternion:
      double qw = qr.w, qx = qr.x, qy = qr.y, qz = qr.z;
      double ax_b = a.x, ay_b = a.y, az_b = a.z;

      // Rotate body accel to ENU: a_enu = R(q) * a_body
      double ax_enu = (1 - 2*(qy*qy + qz*qz)) * ax_b + 2*(qx*qy - qw*qz) * ay_b + 2*(qx*qz + qw*qy) * az_b;
      double ay_enu = 2*(qx*qy + qw*qz) * ax_b + (1 - 2*(qx*qx + qz*qz)) * ay_b + 2*(qy*qz - qw*qx) * az_b;
      double az_enu = 2*(qx*qz - qw*qy) * ax_b + 2*(qy*qz + qw*qx) * ay_b + (1 - 2*(qx*qx + qy*qy)) * az_b;

      // Subtract gravity (ENU: gravity is [0, 0, -9.80665])
      constexpr double GRAVITY = 9.80665;
      az_enu -= GRAVITY;

      // ENU -> NED: x_NED = y_ENU, y_NED = x_ENU, z_NED = -z_ENU
      px4_msg.ax = static_cast<float>(ay_enu);
      px4_msg.ay = static_cast<float>(ax_enu);
      px4_msg.az = static_cast<float>(-az_enu);
    }

    local_pose_pub_->publish(px4_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr local_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr local_acc_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pose_pub_;

  std::string local_pose_topic_ros2_;
  std::string local_pose_topic_ros1_;
  std::string local_vel_topic_ros1_;
  std::string local_acc_topic_ros1_;

  geometry_msgs::msg::TwistStamped last_velocity_;
  bool has_velocity_ = false;

  sensor_msgs::msg::Imu last_imu_;
  bool has_imu_ = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPoseConversion>());
  rclcpp::shutdown();
  return 0;
}
