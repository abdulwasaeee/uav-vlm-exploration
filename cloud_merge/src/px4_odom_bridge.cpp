// px4_odom_bridge.cpp
// Converts PX4 VehicleOdometry (raw NED, pose_frame=1, FRD body)
// → ROS ENU/FLU convention and publishes:
//   - nav_msgs/Odometry  on /drone/odom
//   - dynamic TF: odom → base_link
//   - static  TF: map  → odom  (identity until SLAM is added)
//
// NED → ENU position:   x_enu =  y_ned,  y_enu =  x_ned,  z_enu = -z_ned
// FRD → FLU quaternion: post-multiply by R(pi around X) then R(pi/2 around Z)
//   Equivalent fixed rotation: q_enu = q_rot * q_ned
//   where q_rot = RPY(pi, 0, pi/2)

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

class Px4OdomBridge : public rclcpp::Node
{
public:
  Px4OdomBridge() : Node("px4_odom_bridge")
  {
    tf_br_     = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_static_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Static map → odom (identity; replace with SLAM/VIO output later)
    geometry_msgs::msg::TransformStamped map_odom;
    map_odom.header.stamp    = get_clock()->now();
    map_odom.header.frame_id = "map";
    map_odom.child_frame_id  = "odom";
    map_odom.transform.rotation.w = 1.0;
    tf_static_->sendTransform(map_odom);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/drone/odom", 10);



    sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        rclcpp::QoS(10).best_effort(),
        std::bind(&Px4OdomBridge::odom_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Px4OdomBridge ready (NED→ENU, pose_frame=1)");
  }

private:
  void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // PX4 timestamp is in microseconds
    rclcpp::Time stamp(static_cast<int64_t>(msg->timestamp) * 1000LL, RCL_ROS_TIME);

    // ── Position: NED → ENU ─────────────────────────────────────────
    double px =  msg->position[1];   // east  = y_ned
    double py =  msg->position[0];   // north = x_ned  (ROS y = north)
    double pz = -msg->position[2];   // up    = -z_ned

    // ── Velocity: NED → ENU ─────────────────────────────────────────
    double vx =  msg->velocity[1];
    double vy =  msg->velocity[0];
    double vz = -msg->velocity[2];

    // ── Quaternion: PX4 NED/FRD → ROS ENU/FLU ──────────────────────
    // PX4 q = [w, x, y, z] in NED/FRD frame.
    // NED→ENU + FRD→FLU is achieved by this component remap:
    //   q_enu.w =  q_ned.w
    //   q_enu.x =  q_ned.y   (ENU x = NED y)
    //   q_enu.y =  q_ned.x   (ENU y = NED x)
    //   q_enu.z = -q_ned.z   (ENU z = -NED z)
    tf2::Quaternion q_enu(
         msg->q[2],   // x_enu = y_ned  → q[2]
         msg->q[1],   // y_enu = x_ned  → q[1]
        -msg->q[3],   // z_enu = -z_ned → -q[3]
         msg->q[0]);  // w unchanged    → q[0]
    q_enu.normalize();

    // ── TF: odom → base_link ────────────────────────────────────────
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id  = "base_link";
    tf.transform.translation.x = px;
    tf.transform.translation.y = py;
    tf.transform.translation.z = pz;
    tf.transform.rotation.x = q_enu.x();
    tf.transform.rotation.y = q_enu.y();
    tf.transform.rotation.z = q_enu.z();
    tf.transform.rotation.w = q_enu.w();
    tf_br_->sendTransform(tf);

    // ── Odometry msg ────────────────────────────────────────────────
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";
    odom.pose.pose.position.x    = px;
    odom.pose.pose.position.y    = py;
    odom.pose.pose.position.z    = pz;
    odom.pose.pose.orientation.x = q_enu.x();
    odom.pose.pose.orientation.y = q_enu.y();
    odom.pose.pose.orientation.z = q_enu.z();
    odom.pose.pose.orientation.w = q_enu.w();
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = vz;
    odom_pub_->publish(odom);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_br_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4OdomBridge>());
  rclcpp::shutdown();
}