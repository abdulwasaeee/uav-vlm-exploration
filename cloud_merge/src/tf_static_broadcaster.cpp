// tf_static_broadcaster.cpp
// Publishes static TF transforms for the full TF tree:
//
//   map → odom          (identity — updated by EKF/VIO later)
//   odom → base_link    (from PX4 odometry — published separately)
//   base_link → tof_array_link   (identity joint in SDF)
//   tof_array_link → tof_N_link  (72° ring, exact poses from SDF)
//
// NOTE: map→odom and odom→base_link are handled dynamically by
//       the px4_odom_bridge node. This node only publishes the
//       static sensor transforms.

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

// Helper: build a TransformStamped from (x,y,z, roll,pitch,yaw)
static geometry_msgs::msg::TransformStamped make_tf(
    const std::string& parent, const std::string& child,
    double x, double y, double z,
    double roll, double pitch, double yaw,
    const rclcpp::Time& t)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp    = t;
  ts.header.frame_id = parent;
  ts.child_frame_id  = child;
  ts.transform.translation.x = x;
  ts.transform.translation.y = y;
  ts.transform.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  ts.transform.rotation.x = q.x();
  ts.transform.rotation.y = q.y();
  ts.transform.rotation.z = q.z();
  ts.transform.rotation.w = q.w();
  return ts;
}

class TfStaticBroadcaster : public rclcpp::Node
{
public:
  TfStaticBroadcaster() : Node("tf_static_broadcaster")
  {
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    auto t = get_clock()->now();

    // ── base_link → tof_array_link (identity joint in SDF) ──────────
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    tfs.push_back(make_tf("base_link", "tof_array_link", 0,0,0, 0,0,0, t));

    // ── tof_array_link → tof_N_link (exact poses from SDF) ──────────
    // tof_0: front       pos( 0.250,  0.000, 0)  yaw=0.0000
    // tof_1: front-left  pos( 0.077,  0.236, 0)  yaw=1.2566 (~72°)
    // tof_2: rear-left   pos(-0.202,  0.147, 0)  yaw=2.5133 (~144°)
    // tof_3: rear-right  pos(-0.202, -0.147, 0)  yaw=-2.5133 (~-144°)
    // tof_4: front-right pos( 0.077, -0.236, 0)  yaw=-1.2566 (~-72°)
    struct TofPose { double x,y,z,yaw; };
    const std::array<TofPose, 5> poses = {{
      { 0.250,  0.000, 0.0,  0.0000},   // tof_0
      { 0.077,  0.236, 0.0,  1.2566},   // tof_1
      {-0.202,  0.147, 0.0,  2.5133},   // tof_2
      {-0.202, -0.147, 0.0, -2.5133},   // tof_3
      { 0.077, -0.236, 0.0, -1.2566},   // tof_4
    }};

    for (int i = 0; i < 5; ++i) {
      auto& p = poses[i];
      tfs.push_back(make_tf(
          "tof_array_link",
          "tof_" + std::to_string(i) + "_link",
          p.x, p.y, p.z,
          0.0, 0.0, p.yaw, t));
    }

    // ── base_link → rgbd_cam_link ────────────────────────────────────
    // SDF pose: <pose>0.2 0 0.02 0 0 0</pose>
    tfs.push_back(make_tf("base_link", "rgbd_cam_link", 0.2, 0.0, 0.02, 0,0,0, t));

    // ── base_link → bottom_cam_link ──────────────────────────────────
    // SDF pose: <pose>0 0 -0.05 0 1.5708 0</pose>  (pitched 90° down)
    tfs.push_back(make_tf("base_link", "bottom_cam_link", 0.0, 0.0, -0.05, 0, M_PI_2, 0, t));

    broadcaster_->sendTransform(tfs);
    RCLCPP_INFO(get_logger(), "Published %zu static transforms", tfs.size());
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfStaticBroadcaster>());
  rclcpp::shutdown();
}